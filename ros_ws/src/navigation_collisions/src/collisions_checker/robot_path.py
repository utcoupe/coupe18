#!/usr/bin/python
import math
import rospy
from map_shapes import Rect, Circle
from map_classes import Point, Position, MapObstacle
from robot_path_collisions import PathChecker


class PathRect(MapObstacle):
    def __init__(self, shape, position):
        super(PathRect, self).__init__(shape, position)

    def corners(self):
        corners = []
        corners.append(Point(0, 0))

    def intersects(self, obstacle):
        if isinstance(obstacle.Shape, Point):
            return self.point_in_self(obstacle)
        elif isinstance(obstacle.Shape, Circle):
            return self.circle_touching_self(obstacle)
        else:
            rospy.logerr("Collision test between rect and object type '{}' not implemented.".format(type(obstacle.Shape)))
        return False

    def point_in_self(self, point): # Calculs persos, a tester #PS22
        p_prime = (point.X * math.cos(self.Position.A) + point.Y * math.sin(self.Position.A),
                   point.Y * math.cos(self.Position.A) - point.X * math.sin(self.Position.A))
        return abs(self.Position.X - p_prime[0]) <= self.Shape.Width / 2.0 and \
               abs(self.Position.Y - p_prime[1]) <= self.Shape.Height / 2.0

    def rect_touching_self(self, rect, check_other = True):
        for p in rect.corners(): # Check if one corner of rect ins inside this one
            if self.point_in_self(p):
                return True
        if check_other and rect.rect_touching_self(self, check_other = False): # Do the other way around
            return True
        return False

    def circle_touching_self(self, circle):
        p_prime = (circle.Position.X * math.cos(self.Position.A) + circle.Position.Y * math.sin(self.Position.A),
                   circle.Position.Y * math.cos(self.Position.A) - circle.Position.X * math.sin(self.Position.A))

    def distanceToCollision(self, obstacle):
        return 0.6 # TODO NotImplemented


class PathCircle(MapObstacle):
    def __init__(self, shape, position):
        super(PathCircle, self).__init__(shape, position)

    def toEctagon(self):
        return NotImplementedError

    def distanceToCollision(self, obstacle):
        return 0.35 # TODO Not implemented



class RobotPath(object):
    def __init__(self, waypoints = None):
        self.Waypoints = []
        self.checker = PathChecker()
        self.updateWaypoints(waypoints)

    def hasPath(self):
        return len(self.Waypoints) != 0

    def updateWaypoints(self, waypoints_tuples):
        if waypoints_tuples:
            self.Waypoints = waypoints_tuples

    def toShapes(self, robot):
        if len(self.Waypoints) > 1:
            shapes = []
            for i in range(1, len(self.Waypoints)):
                # Creating a rectangle with the robot's width between each waypoint
                p_w = robot.Position if i == 1 else self.Waypoints[i - 1]
                w   = self.Waypoints[i]

                d = math.sqrt( (w.X - p_w.X) ** 2 + (w.Y - p_w.Y) ** 2)
                pos = Position((w.X + p_w.X) / 2.0, (w.Y + p_w.Y) / 2.0, angle = math.atan((w.Y - p_w.Y) / (w.X - p_w.X)))

                if isinstance(robot.Shape, Rect):
                    width = robot.Shape.Width
                elif isinstance(robot.Shape, Circle):
                    width = robot.Shape.Radius * 2.0
                else:
                    raise ValueError("Robot shape type not supported.")
                shapes.append(PathRect(Rect(d, width), pos))

                # TODO Create a circle at each waypoint position
            return shapes
        else:
            rospy.logerr("Path can't create shapes : less than two waypoints in path")

    def checkCollisions(self, robot, obstacles):
        path_shapes = self.toShapes(robot)
        return self.checker.checkCollisions(robot, path_shapes, obstacles)
