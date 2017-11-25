#!/usr/bin/python
import math
import rospy
from map_classes import Position, RectObstacle, CircleObstacle, Rect, Circle
from robot_path_collisions import PathChecker


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
                angle = math.atan((w.Y - p_w.Y) / (w.X - p_w.X))
                pos = Position((w.X + p_w.X) / 2.0, (w.Y + p_w.Y) / 2.0, angle = angle)

                if isinstance(robot.Shape, Rect):
                    width = robot.Shape.Width
                    height = robot.Shape.Height
                elif isinstance(robot.Shape, Circle):
                    width = robot.Shape.Radius * 2.0
                    height = robot.Shape.Radius * 2.0
                else:
                    raise ValueError("Robot shape type not supported.")
                shapes.append(RectObstacle(Rect(d, width), pos))

                if i == len(self.Waypoints) - 1:
                    shapes.append(RectObstacle(Rect(height, width), Position(w.X, w.Y, angle)))
                else:
                    shapes.append(CircleObstacle(Circle(width / 2.0), Position(w.X, w.Y)))
            return shapes
        else:
            rospy.logerr("Path can't create shapes : less than two waypoints in path")

    def checkCollisions(self, robot, obstacles):
        path_shapes = self.toShapes(robot)
        return self.checker.checkCollisions(robot, path_shapes, obstacles)
