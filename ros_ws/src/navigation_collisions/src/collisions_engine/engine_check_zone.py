import math
import rospy
from engine_constants import CollisionThresholds
from engine_shapes import RectObstacle, CircleObstacle
from engine_shapes_attrib import Position
from engine import CollisionsResolver


class CheckZone(object):
    def __init__(self, width, height, collision_level):
        self._width = width
        self._height = height
        self.collision_level = collision_level

    def get_shapes(self, robot_pos):
        return []

    def check_collisions(self, robot_pos, obstacles):
        return CollisionsResolver.find_collisions(self.get_shapes(robot_pos), obstacles)


class MainCheckZone(CheckZone):
    def __init__(self, width, height, collision_level):
        super(MainCheckZone, self).__init__(width, height, collision_level)

    def get_shapes(self, robot_pos, velocity):
        w, h = self._height + CollisionThresholds.get_stop_distance(velocity.linear), self._width
        l = w / 2.0 - self._height / 2.0
        side_a = math.pi if velocity.linear < 0 else 0
        return [RectObstacle(Position(robot_pos.x + l * math.cos(robot_pos.a + side_a),
                                      robot_pos.y + l * math.sin(robot_pos.a + side_a),
                                      robot_pos.a), w, h)]

    def check_collisions(self, robot_pos, robot_vel, obstacles):
        return CollisionsResolver.find_collisions(self.get_shapes(robot_pos, robot_vel), obstacles)


class PathCheckZone(CheckZone):
    def __init__(self, width, height, collision_level):
        super(PathCheckZone, self).__init__(width, height, collision_level)
        self._waypoints = []

    def update_path(self, new_waypoints):
        if isinstance(new_waypoints, list) and len(new_waypoints) > 0:
            self._waypoints = new_waypoints
        else:
            rospy.logerr("Trying to update the robot path with an invalid veriable type.")

    def get_shapes(self, robot_pos):
        if len(self._waypoints) >= 1:
            shapes = []
            for i in range(1, len(self._waypoints)):
                # Creating a rectangle with the robot's width between each waypoint
                p_w = robot_pos if i == 1 else self._waypoints[i - 1]
                w   = self._waypoints[i]

                if p_w.x != w.x and p_w.y != w.y:
                    d = math.sqrt( (w.x - p_w.x) ** 2 + (w.y - p_w.y) ** 2)
                    angle = math.atan((w.y - p_w.y) / (w.x - p_w.x))
                    pos = Position((w.x + p_w.x) / 2.0, (w.y + p_w.y) / 2.0, angle = angle)

                    shapes.append(RectObstacle(pos, d, self._width))
                    if i == len(self._waypoints) - 1:
                        shapes.append(RectObstacle(Position(w.x, w.y, angle), self._height, self._width))
                    else:
                        shapes.append(CircleObstacle(Position(w.x, w.y), self._width / 2.0))
            return shapes
        else:
            return []
