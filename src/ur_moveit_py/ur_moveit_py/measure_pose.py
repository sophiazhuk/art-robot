"""One-off: move to a measured joint configuration and print the pen-tip pose.

Used to find the table height for the planning-scene table in drawing.py:
freedrive the arm until the sharpie just touches the table, read the joint
values off the pendant, put them in MEASURED_DEGREES below, then run this.
It moves there in sim/on the arm and prints the pen-tip pose in base_link
(same frame, and same PEN_LENGTH-corrected tip, as everywhere else in this
package), including the z height to use for the table.
"""
import math

import rclpy
from moveit_msgs.msg import MoveItErrorCodes

from ur_moveit_py.drawing import Drawer

# Joint values from freedrive, sharpie touching the table (degrees, pendant order).
MEASURED_DEGREES = {
    "shoulder_pan_joint": -52.35,
    "shoulder_lift_joint": -35.88,
    "elbow_joint": 52.55,
    "wrist_1_joint": -107.00,
    "wrist_2_joint": -86.47,
    "wrist_3_joint": 89.09,
}
MEASURED_JOINTS = {name: math.radians(deg) for name, deg in MEASURED_DEGREES.items()}


def main():
    rclpy.init()
    node = Drawer("measure_pose")
    log = node.get_logger()
    try:
        node.add_pen()
        log.info("Moving to measured joint pose...")
        code = node.move_to_ready(MEASURED_JOINTS)
        if code != MoveItErrorCodes.SUCCESS:
            log.error(f"Could not reach measured pose, MoveIt error code {code}")
            return
        node.settle()

        pose = node.current_pose()
        log.info(
            f"Pen tip: x={pose.position.x:.4f} y={pose.position.y:.4f} z={pose.position.z:.4f} "
            f"qx={pose.orientation.x:.4f} qy={pose.orientation.y:.4f} "
            f"qz={pose.orientation.z:.4f} qw={pose.orientation.w:.4f}"
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
