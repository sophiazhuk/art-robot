import copy
import math

from geometry_msgs.msg import Pose

from ur_moveit_py.drawing import draw

RADIUS = 0.04              # circle radius, meters
SEGMENTS = 72              # straight segments around the circle (5 degrees each)


def circle_waypoints(start: Pose) -> list[Pose]:
    """Full circle at constant height and orientation, starting and ending at the start.

    The start is the circle's +x edge, so the circle is drawn toward the base (-x).
    """
    cx = start.position.x - RADIUS
    cy = start.position.y
    waypoints = []
    for i in range(1, SEGMENTS + 1):
        theta = 2.0 * math.pi * i / SEGMENTS
        p = copy.deepcopy(start)
        p.position.x = cx + RADIUS * math.cos(theta)
        p.position.y = cy + RADIUS * math.sin(theta)
        waypoints.append(p)
    # close the loop exactly, without floating-point drift
    waypoints[-1].position.x = start.position.x
    waypoints[-1].position.y = start.position.y
    return waypoints


def main():
    draw("draw_circle", circle_waypoints, "circle")


if __name__ == "__main__":
    main()
