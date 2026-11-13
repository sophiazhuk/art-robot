#!/usr/bin/env python3
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
import rclpy
from rclpy.logging import get_logger
from moveit.planning import MoveItPy, PlanRequestParameters
import time

def main(args=None):
    # Initialize ROS 2 context
    rclpy.init()
    logger = get_logger("ur_moveit_py_test")
    logger.info("Initializing MoveItPy Node...")

    # set controller
    active_controllers = ["scaled_joint_trajectory_controller"]

    # Instantiate MoveItPy
    # This automatically processes the robot_description and parameters from the launch context
    ur3e = MoveItPy(node_name="ur3e_moveit_py_node")
    time.sleep(2.0) # give time for action client to connect to Gazebo

    # access the primary arm planning group
    arm = ur3e.get_planning_component("ur_manipulator")

    # Tell MoveItPy the table exists (size and offset from the lab README)
    psm = ur3e.get_planning_scene_monitor()
    with psm.read_write() as scene:
        table = CollisionObject()
        table.header.frame_id = "base_link"
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
        scene.apply_collision_object(table)
        scene.current_state.update()

    # Command a move to the 'home' pose first to ensure a known start state
    logger.info("Setting goal state to 'home'...")
    arm.set_start_state_to_current_state()
    arm.set_goal_state(configuration_name="home")

    logger.info("Planning trajectory to 'home'...")
    plan_to_home = arm.plan()

    if plan_to_home:
        logger.info("Executing movement to 'home'...")
        ur3e.execute(plan_to_home.trajectory, controllers=active_controllers)
    else:
        logger.error("Planning to 'home' failed!")

    # Plan and move to 'up' (another standard named target in the UR SRDF)
    logger.info("Setting goal state to 'up'...")
    arm.set_start_state_to_current_state()
    arm.set_goal_state(configuration_name="up")

    logger.info("Planning trajectory to 'up'...")
    plan_to_up = arm.plan()

    if plan_to_up:
        logger.info("Executing movement to 'up'...")
        ur3e.execute(plan_to_up.trajectory, controllers=active_controllers)
        logger.info("Movement complete!")
    else:
        logger.error("Planning to 'up' failed!")

    # 6. Shutdown cleanly
    rclpy.shutdown()


if __name__ == "__main__":
    main()
