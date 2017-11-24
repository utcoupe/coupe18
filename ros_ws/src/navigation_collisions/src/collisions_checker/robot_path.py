#!/usr/bin/python
import math
import rospy
from map_classes import Point, Position, MapObstacle, RectObstacle, CircleObstacle, Rect, Circle
from robot_path_collisions import PathChecker

from map import Map

class Intersections(object):
    def intersects(self, obstacle):
        if isinstance(obstacle.Shape, Point):
            return self.point_in_self(obstacle)
        elif isinstance(obstacle.Shape, Circle):
            return self.circle_touching_self(obstacle)
        else:
            rospy.logerr("Collision test between rect and object type '{}' not implemented.".format(type(obstacle.Shape)))
        return False

    def segments_intersect(self, s1, s2):
        # https://stackoverflow.com/questions/3838329/how-can-i-check-if-two-segments-intersect
        ccw = lambda a, b, c: (c.Y-a.Y) * (b.X-a.X) > (b.Y-a.Y) * (c.X-a.X)
        return ccw(s1[0],s2[0],s2[1]) != ccw(s1[1],s2[0],s2[1]) and ccw(s1[0],s1[1],s2[0]) != ccw(s1[0],s1[1],s2[1])

    def point_in_rect(self, rect, point): # Calculs persos, a tester #PS22
        p_prime = (point.X * math.cos(rect.Position.A) + point.Y * math.sin(rect.Position.A),
                   point.Y * math.cos(rect.Position.A) - point.X * math.sin(rect.Position.A))
        return abs(rect.Position.X - p_prime[0]) <= rect.Shape.Width / 2.0 and \
               abs(rect.Position.Y - p_prime[1]) <= rect.Shape.Height / 2.0

    def rects_intersect(self, rect1, rect2):
        # for p in rect1.corners(): # Check if one corner of rect ins inside this one
        #     if self.point_in_rect(rect2, p):
        #         return True
        # for p in rect2.corners(): # Check if one corner of rect is inside the other one
        #     if self.point_in_rect(rect1, p):
        #         return True
        for s1 in rect1.segments(): # If not, check if segments intersect
            for s2 in rect2.segments():
                if self.segments_intersect(s1, s2):
                    return True
        return False

    def circle_touching_self(self, circle):
        p_prime = (circle.Position.X * math.cos(self.Position.A) + circle.Position.Y * math.sin(self.Position.A),
                   circle.Position.Y * math.cos(self.Position.A) - circle.Position.X * math.sin(self.Position.A))





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
                    shapes.append(RectObstacle(Rect(height, width), Position(w.X, w.Y, angle + 2)))
                else:
                    shapes.append(CircleObstacle(Circle(width / 2.0), Position(w.X, w.Y)))
            return shapes
        else:
            rospy.logerr("Path can't create shapes : less than two waypoints in path")

    def checkCollisions(self, robot, obstacles):
        path_shapes = self.toShapes(robot)
        Map.LidarObjects = []

        i = Intersections()
        print "intersection returned " + str(i.rects_intersect(path_shapes[2], Map.BeltPoints[1]))
        return []#return self.checker.checkCollisions(robot, path_shapes, obstacles)
