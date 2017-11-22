#!/usr/bin/python
import math
import rospy
from map_shapes import Rect, Circle
from map_classes import Position, MapObstacle
from robot_path_collisions import PathChecker


class PathRect(MapObstacle):
    def __init__(self, shape, position):
        super(PathRect, self).__init__(shape, position)

    def distanceToCollision(self, obstacle):
        return 0.6 # TODO NotImplemented


class PathCircle(MapObstacle):
    def __init__(self, shape, position):
        super(PathCircle, self).__init__(shape, position)

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
                p_w = robot.Position.tuple2() if i == 1 else self.Waypoints[i - 1]
                w   = self.Waypoints[i]

                d = math.sqrt( (w[0] - p_w[0]) ** 2 + (w[1] - p_w[1]) ** 2)
                pos = Position((w[0] + p_w[0]) / 2.0, (w[1] + p_w[1]) / 2.0, angle = math.atan((w[1] - p_w[1]) / (w[0] - p_w[0])))

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
