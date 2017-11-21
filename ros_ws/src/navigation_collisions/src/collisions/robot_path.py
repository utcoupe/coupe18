#!/usr/bin/python
import math
import rospy
from map_classes import *
from path_checker import PathChecker

class Waypoint(Position):
    def __init__(self, x, y):
        self.X = x
        self.Y = y


class PathRect(MapObstacle):
    def __init__(self, width, height):
        pass

    def distanceToCollision(self, obstacle):
        obs_pos = obstacle.Position


class PathCircle(MapObstacle):
    pass


class RobotPath(object):
    def __init__(self, waypoints = None):
        self.Waypoints = []
        self.checker = PathChecker()
        self.updateWaypoints(waypoints)

    def hasPath(self):
        return len(self.Waypoints) != 0

    def updateWaypoints(self, waypoints_list):
        if waypoints_list:
            for w in waypoints_list:
                self.Waypoints.append(Waypoint(w[0], w[1]))

    def toShapes(self, robot):
        if len(self.Waypoints) > 1:
            shapes = []
            for i in range(1, len(self.Waypoints)):
                # Creating a rectangle with the robot's width between each waypoint
                p_w = robot.Position if i == 1 else self.Waypoints[i - 1]
                w   = self.Waypoints[i]
                print p_w.X

                d = math.sqrt( (w.X - p_w.X) ** 2 + (w.Y - p_w.Y) ** 2)
                pos = Position((w.X + p_w.X) / 2.0, (w.Y + p_w.Y) / 2.0, angle = math.atan((w.Y - p_w.Y) / (w.X - p_w.X)))

                shapes.append(MapObstacle(Rect(d, robot.Shape.Width), pos, velocity = None))

                # TODO Create a circle at each waypoint position
            return shapes
        else:
            rospy.logerr("Path can't create shapes : less than two waypoints in path")

    def checkCollisions(self, robot, obstacles):
        path_shapes = self.toShapes(robot)
        return self.checker.checkCollisions(robot, path_shapes, obstacles)
