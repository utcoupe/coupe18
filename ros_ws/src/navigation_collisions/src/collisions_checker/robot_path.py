#!/usr/bin/python
import math
import rospy
from map_classes import Position, RectObstacle, CircleObstacle
from collisions import Collision, CollisionLevel, CollisionsResolver


class RobotPath(object):
    def __init__(self, waypoints = None):
        self.Waypoints = []
        self.updateWaypoints(waypoints)

    def hasPath(self):
        return len(self.Waypoints) != 0

    def updateWaypoints(self, waypoints_tuples):
        if waypoints_tuples:
            self.Waypoints = waypoints_tuples

    def toShapes(self, robot):
        if len(self.Waypoints) >= 1:
            shapes = []
            for i in range(1, len(self.Waypoints)):
                # Creating a rectangle with the robot's width between each waypoint
                p_w = robot.Position if i == 1 else self.Waypoints[i - 1]
                w   = self.Waypoints[i]

                if p_w.X != w.X and p_w.Y != w.Y: # TODO moche
                    d = math.sqrt( (w.X - p_w.X) ** 2 + (w.Y - p_w.Y) ** 2)
                    angle = math.atan((w.Y - p_w.Y) / (w.X - p_w.X))
                    pos = Position((w.X + p_w.X) / 2.0, (w.Y + p_w.Y) / 2.0, angle = angle)

                    shapes.append(RectObstacle(pos, d, robot.Width))
                    if i == len(self.Waypoints) - 1:
                        shapes.append(RectObstacle(Position(w.X, w.Y, angle), robot.Height, robot.Width))
                    else:
                        shapes.append(CircleObstacle(Position(w.X, w.Y), robot.Width / 2.0))
            return shapes
        else:
            rospy.logerr("Path can't create shapes : less than two waypoints in path")
            return []

    def checkCollisions(self, robot, obstacles):
        path_shapes = self.toShapes(robot)
        i = CollisionsResolver()

        collisions = []
        distance_to_collision = 0.0
        for path_segment in path_shapes:
            for obstacle in obstacles:
                if len([o for o in collisions if o.Obstacle == obstacle]) != 0:
                    continue # Ignore already found collision obstacles

                # Check if obstacle intersects with one of the path shapes
                if i.intersect(path_segment, obstacle):
                    collisions.append(Collision(CollisionLevel.LEVEL_DANGER, obstacle, distance_to_collision))
            distance_to_collision += path_segment.Width if str(path_segment) == "rect" else 0
        return collisions
