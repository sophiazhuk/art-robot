import copy

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformListener

GROUP = "ur_manipulator"   # UR MoveIt planning group
BASE = "base_link"         # frame for waypoints and the table
TIP = "tool0"              # tool flange; becomes the pen tip later
SIDE = 0.08                # square side length, meters
STEP = 0.005               # Cartesian interpolation step, meters
SPEED = 0.25               # fraction of planned speed (0.25 = 4x slower)


class DrawSquare(Node):
    def __init__(self):
        super().__init__("draw_square")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.scene_client = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.cartesian_client = self.create_client(GetCartesianPath, "/compute_cartesian_path")
        self.execute_client = ActionClient(self, ExecuteTrajectory, "/execute_trajectory")

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
        box.dimensions = [2.0, 2.0, 0.01]
        pose = Pose()
        pose.position.z = -0.01
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

    def current_pose(self) -> Pose:
        """Where the tool flange is right now."""
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
        return pose

    def square_waypoints(self, start: Pose) -> list[Pose]:
        """Four corners at constant height and orientation, ending at the start."""
        waypoints = []
            for dx, dy in [(-SIDE, 0.0), (-SIDE, -SIDE), (0.0, -SIDE), (0.0, 0.0)]:
            p = copy.deepcopy(start)
            p.position.x += dx
            p.position.y += dy
            waypoints.append(p)
        return waypoints

    def plan(self, waypoints: list[Pose]):
        req = GetCartesianPath.Request()
        req.header.frame_id = BASE
        req.start_state.is_diff = True   # start from the arm's current state
        req.group_name = GROUP
        req.link_name = TIP
        req.waypoints = waypoints
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


def main():
    rclpy.init()
    node = DrawSquare()
    log = node.get_logger()
    try:
        node.add_table()
        start = node.current_pose()
        log.info(f"Start: x={start.position.x:.3f} y={start.position.y:.3f} z={start.position.z:.3f}")

        response = node.plan(node.square_waypoints(start))
        log.info(f"Planned fraction: {response.fraction:.3f}")
        if response.fraction < 0.999:
            log.error("Could not plan the full square. Move the start pose or shrink SIDE.")
            return

        code = node.execute(node.slow_down(response.solution, SPEED))
        if code == 1:  # MoveItErrorCodes.SUCCESS
            log.info("Square complete.")
        else:
            log.error(f"Execution failed, MoveIt error code {code}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()