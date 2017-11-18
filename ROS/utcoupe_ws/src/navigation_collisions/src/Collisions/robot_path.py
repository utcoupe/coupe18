#!/usr/bin/python
import math
import rospy
from map_classes import *
from path_checker import PathChecker

class Waypoint(Position):
    pass


class PathRect(Obstacle):
    def __init__(self, width, height):
        pass

    def distanceToCollision(self, obstacle):
        pass


class RobotPath(object):
    def __init__(self):
        self.Waypoints = []
        self.checker = PathChecker()

    def hasPath(self):
        return self.Waypoints is not None

    def updateWaypoints(self, waypoints_list):
        self.Waypoints = waypoints_list

    def toShapes(self, robot):
        if len(self.Waypoints) > 1:
            shapes = []
            for i in range(1, len(self.Waypoints)):
                # Creating a rectangle with the robot's width between each waypoint
                p_w = robot.Position if i == 1 else self.Waypoints[i - 1]
                w   = self.Waypoints[i]

                d = math.sqrt( (w.X - p_w.X) ** 2 + (w.Y - p_w.Y) ** 2)
                pos = Position((w.X + p_w.X) / 2.0, (w.Y + p_w.Y) / 2.0, angle = math.atan((w.Y - p_w.Y) / (w.X - p_w.X)))

                shapes.append(MapObstacle(pos, Rect(d, robot.Shape.Width), velocity = None))

                # TODO Create a circle at each waypoint position
            return shapes
        else:
            rospy.logerr("Path can't create shapes : less than two waypoints in path")

    def checkCollisions(self, robot, obstacles):
        self.checker.checkCollisions(robot, self.toShapes(), obstacles)
