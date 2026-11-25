"""Shared drawing code: table, ready pose, Cartesian planning, execution.

Each shape module only defines its waypoints and calls `draw()`.
"""
import copy

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    AttachedCollisionObject,
    CollisionObject,
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    PlanningScene,
)
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker

GROUP = "ur_manipulator"   # UR MoveIt planning group
BASE = "base_link"         # frame for waypoints and the table
TIP = "tool0"              # tool flange; the link MoveIt actually knows about
STEP = 0.005               # Cartesian interpolation step, meters
SPEED = 0.25               # fraction of planned speed (0.25 = 4x slower)

# Sharpie + holder attached to tool0, measured along tool0's own +Z axis
# (its pointing direction). MoveIt has no idea this exists, so every pose
# this file hands out to callers (current_pose(), waypoints) is in terms of
# this pen tip; plan() converts back to tool0 right before asking MoveIt to
# plan, since tool0 is the only link MoveIt can do FK/IK for.
PEN_LENGTH = 0.1524        # 6 inches, in meters

# Pen model for RViz and collision checking (add_pen()). The collision
# cylinder stops PEN_CLEARANCE short of the real tip: the tip is supposed to
# touch the table while drawing, and MoveIt would otherwise reject every
# drawing path as a collision. The exact tip is shown as a red sphere marker
# instead, which is visual only (RViz: Add > By topic > /visualization_marker).
PEN_RADIUS = 0.01          # meters; Sharpie + holder, roughly
PEN_CLEARANCE = 0.01       # meters short of the real tip
TIP_MARKER_DIAMETER = 0.01 # meters

# Table collision box: a thin slab whose top face sits at TABLE_Z, the real
# table surface height in base_link. -0.005 is the old guess (table ~ flush
# with the robot's own base plane); replace it with the z printed by
# `pixi run measure_pose` once you've measured it with the sharpie touching
# the table.
TABLE_Z = -0.005           # meters, base_link
TABLE_THICKNESS = 0.01     # meters; only the top face matters for collision

# Pen-down ready pose: elbow bent, tool pointing straight down.
# tool0 ends up at about (0.30, 0.13, 0.30) m in base_link, so the pen tip
# (6 inches further along tool0's +Z, i.e. further down) is at about
# (0.30, 0.13, 0.15) m. Shapes are drawn from here toward the base (-x),
# which keeps them well inside the arm's reach.
# The sim spawns the arm straight up and fully stretched, where no
# Cartesian shape is plannable, so we always move here first.
READY_JOINTS = {
    "shoulder_pan_joint": 0.0,
    "shoulder_lift_joint": -1.57,
    "elbow_joint": 1.57,
    "wrist_1_joint": -1.57,
    "wrist_2_joint": -1.57,
    "wrist_3_joint": 0.0,
}
JOINT_TOL = 0.01           # rad, tolerance on each ready-pose joint


def advance_along_z(pose: Pose, length: float) -> Pose:
    """Return a copy of `pose` moved `length` meters along its own +Z axis."""
    x, y, z, w = pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
    out = copy.deepcopy(pose)
    out.position.x += 2.0 * length * (x * z + w * y)
    out.position.y += 2.0 * length * (y * z - w * x)
    out.position.z += length * (1.0 - 2.0 * (x * x + y * y))
    return out


class Drawer(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.scene_client = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.cartesian_client = self.create_client(GetCartesianPath, "/compute_cartesian_path")
        self.execute_client = ActionClient(self, ExecuteTrajectory, "/execute_trajectory")
        self.move_group_client = ActionClient(self, MoveGroup, "/move_action")
        self.marker_pub = self.create_publisher(Marker, "/visualization_marker", 10)

    def call(self, client, request, name):
        if not client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(f"{name} unavailable. Is sim (or moveit) running?")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def add_table(self):
        """Add the table to move_group's planning scene (visible in RViz)."""
        table = CollisionObject()
        table.header.frame_id = BASE
        table.id = "table"
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [2.0, 2.0, TABLE_THICKNESS]
        pose = Pose()
        pose.position.z = TABLE_Z - TABLE_THICKNESS / 2.0
        pose.orientation.w = 1.0
        table.primitives.append(box)
        table.primitive_poses.append(pose)
        table.operation = CollisionObject.ADD

        req = ApplyPlanningScene.Request()
        req.scene = PlanningScene()
        req.scene.is_diff = True
        req.scene.world.collision_objects.append(table)
        if not self.call(self.scene_client, req, "/apply_planning_scene").success:
            raise RuntimeError("Failed to add table to planning scene")

    def add_pen(self):
        """Attach the pen to tool0 in the planning scene, and mark its tip.

        The cylinder moves with the arm in RViz and is collision-checked by
        MoveIt. The tip marker is locked to tool0, so it keeps following the
        arm in RViz after this program exits.
        """
        length = PEN_LENGTH - PEN_CLEARANCE
        pen = AttachedCollisionObject()
        pen.link_name = TIP
        pen.touch_links = [TIP, "wrist_3_link", "flange"]   # holder sits on the flange
        pen.object.header.frame_id = TIP
        pen.object.id = "pen"
        cyl = SolidPrimitive()
        cyl.type = SolidPrimitive.CYLINDER
        cyl.dimensions = [length, PEN_RADIUS]   # [height, radius], centered on its pose
        pose = Pose()
        pose.position.z = length / 2.0          # base of the pen at tool0, pointing +Z
        pose.orientation.w = 1.0
        pen.object.primitives.append(cyl)
        pen.object.primitive_poses.append(pose)
        pen.object.operation = CollisionObject.ADD

        req = ApplyPlanningScene.Request()
        req.scene = PlanningScene()
        req.scene.is_diff = True
        req.scene.robot_state.is_diff = True
        req.scene.robot_state.attached_collision_objects.append(pen)
        if not self.call(self.scene_client, req, "/apply_planning_scene").success:
            raise RuntimeError("Failed to attach pen to planning scene")

        tip = Marker()
        tip.header.frame_id = TIP
        tip.ns = "pen"
        tip.id = 0
        tip.type = Marker.SPHERE
        tip.action = Marker.ADD
        tip.frame_locked = True                 # follow tool0 as the arm moves
        tip.pose.position.z = PEN_LENGTH
        tip.pose.orientation.w = 1.0
        tip.scale.x = tip.scale.y = tip.scale.z = TIP_MARKER_DIAMETER
        tip.color.r, tip.color.a = 1.0, 1.0
        # Re-published every second so RViz picks it up whenever its Marker
        # display subscribes; lifetime 0 keeps it shown after we exit.
        self.create_timer(1.0, lambda: self.marker_pub.publish(tip))
        self.marker_pub.publish(tip)

    def move_to_ready(self, joints: dict[str, float] = READY_JOINTS) -> int:
        """Plan and execute a collision-free joint-space move to `joints`."""
        if not self.move_group_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("/move_action unavailable. Is sim (or moveit) running?")
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = GROUP
        req.start_state.is_diff = True   # start from the arm's current state
        req.num_planning_attempts = 5
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = SPEED
        req.max_acceleration_scaling_factor = SPEED
        constraints = Constraints()
        for name, position in joints.items():
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = position
            jc.tolerance_above = JOINT_TOL
            jc.tolerance_below = JOINT_TOL
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        req.goal_constraints.append(constraints)
        goal.planning_options.plan_only = False   # plan and execute

        send_future = self.move_group_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        handle = send_future.result()
        if not handle.accepted:
            raise RuntimeError("Ready-pose goal rejected")
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return result_future.result().result.error_code.val

    def settle(self, seconds: float = 0.5):
        """Spin briefly so TF catches up with where the arm actually is."""
        deadline = self.get_clock().now() + Duration(seconds=seconds)
        while self.get_clock().now() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def current_pose(self) -> Pose:
        """Where the pen tip is right now (tool0, advanced by PEN_LENGTH)."""
        deadline = self.get_clock().now() + Duration(seconds=5.0)
        while not self.tf_buffer.can_transform(BASE, TIP, Time()):
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.get_clock().now() > deadline:
                raise RuntimeError(f"No TF from {BASE} to {TIP}. Is sim (or driver) running?")
        t = self.tf_buffer.lookup_transform(BASE, TIP, Time())
        pose = Pose()
        pose.position.x = t.transform.translation.x
        pose.position.y = t.transform.translation.y
        pose.position.z = t.transform.translation.z
        pose.orientation = t.transform.rotation
        return advance_along_z(pose, PEN_LENGTH)

    def plan(self, waypoints: list[Pose]):
        """Plan a Cartesian path for the pen tip through `waypoints`.

        MoveIt only knows about tool0, so each pen-tip waypoint is shifted
        back by PEN_LENGTH before being sent to /compute_cartesian_path.
        """
        req = GetCartesianPath.Request()
        req.header.frame_id = BASE
        req.start_state.is_diff = True   # start from the arm's current state
        req.group_name = GROUP
        req.link_name = TIP
        req.waypoints = [advance_along_z(wp, -PEN_LENGTH) for wp in waypoints]
        req.max_step = STEP
        req.avoid_collisions = True
        return self.call(self.cartesian_client, req, "/compute_cartesian_path")

    @staticmethod
    def slow_down(trajectory, speed: float):
        """Stretch the trajectory in time so it runs at `speed` x planned speed."""
        for pt in trajectory.joint_trajectory.points:
            t = (pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9) / speed
            pt.time_from_start.sec = int(t)
            pt.time_from_start.nanosec = int((t - int(t)) * 1e9)
            pt.velocities = [v * speed for v in pt.velocities]
            pt.accelerations = [a * speed * speed for a in pt.accelerations]
        return trajectory

    def execute(self, trajectory) -> int:
        if not self.execute_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("/execute_trajectory unavailable. Is sim (or moveit) running?")
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        send_future = self.execute_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        handle = send_future.result()
        if not handle.accepted:
            raise RuntimeError("Trajectory goal rejected")
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return result_future.result().result.error_code.val


def draw(node_name: str, make_waypoints, label: str):
    """Add table, move to ready pose, then plan and draw make_waypoints(start)."""
    rclpy.init()
    node = Drawer(node_name)
    log = node.get_logger()
    try:
        node.add_table()
        node.add_pen()

        log.info("Moving to ready pose...")
        code = node.move_to_ready()
        if code != MoveItErrorCodes.SUCCESS:
            log.error(f"Could not reach ready pose, MoveIt error code {code}")
            return
        node.settle()

        start = node.current_pose()
        log.info(f"Start: x={start.position.x:.3f} y={start.position.y:.3f} z={start.position.z:.3f}")

        response = node.plan(make_waypoints(start))
        log.info(f"Planned fraction: {response.fraction:.3f}")
        if response.fraction < 0.999:
            log.error(f"Could not plan the full {label}. Move the start pose or shrink it.")
            return

        code = node.execute(node.slow_down(response.solution, SPEED))
        if code == MoveItErrorCodes.SUCCESS:
            log.info(f"{label.capitalize()} complete.")
        else:
            log.error(f"Execution failed, MoveIt error code {code}")
    finally:
        node.destroy_node()
        rclpy.shutdown()
