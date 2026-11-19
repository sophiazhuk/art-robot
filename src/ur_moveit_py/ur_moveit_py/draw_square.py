import copy

from geometry_msgs.msg import Pose

from ur_moveit_py.drawing import draw

SIDE = 0.08                # square side length, meters


def square_waypoints(start: Pose) -> list[Pose]:
    """Four corners at constant height and orientation, ending at the start."""
    waypoints = []
    for dx, dy in [(-SIDE, 0.0), (-SIDE, -SIDE), (0.0, -SIDE), (0.0, 0.0)]:
        p = copy.deepcopy(start)
        p.position.x += dx
        p.position.y += dy
        waypoints.append(p)
    return waypoints


def main():
    draw("draw_square", square_waypoints, "square")


if __name__ == "__main__":
    main()
