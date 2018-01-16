#!/usr/bin/python
import math
import rospy

from collisions_shapes import Position, RectObstacle, CircleObstacle
from engine_constants import CollisionExplorerMode


class Collision(object):
    def __init__(self, collision_level, obstacle, approx_distance):
        self.Level = collision_level
        self.Obstacle = obstacle
        self.ApproxDistance = approx_distance # Distance between the centers of the robot and obstacle (poor precision)


class CollisionsExplorer(object):
    def find_collisions(self, robot_shapes, obstacles_shapes):
        collisions = []
        for robot_shape in robot_shapes:
            for obstacles_shape in obstacles_shapes:
                collisions.append(None) # TODO
        return collisions


class CollisionsResolver(object):
    def intersect(self, obs1, obs2):
        types = (str(obs1), str(obs2))
        if types[0] == 'rect' and types[1] == 'rect':
            return self._rects_intersect(obs1, obs2)
        elif types[0] == 'circle' and types[1] == 'circle':
            return self._circles_intersect(obs1, obs2)
        elif types[0] == 'point' and types[1] == 'point':
            return False
        elif "rect" in types and "circle" in types:
            if types[0] == "rect":
                return self._rect_intersects_circle(obs1, obs2)
            return self._rect_intersects_circle(obs2, obs1)
        elif "rect" in types and "point" in types:
            if types[0] == "rect":
                return self._point_in_rect(obs1, obs2)
            return self._rect_intersects_circle(obs2, obs1)
        elif "circle" in types and "point" in types:
            if types[0] == "point":
                return self._point_in_rect(obs1, obs2)
            return self._rect_intersects_circle(obs2, obs1)
        else:
            rospy.logerr("Collisions couldn't check collision between objects of type '{}' and '{}'.".format(types[0], types[1]))

    def _segments_intersect(self, s1, s2):
        # https://stackoverflow.com/questions/3838329/how-can-i-check-if-two-segments-intersect
        ccw = lambda a, b, c: (c.Y-a.Y) * (b.X-a.X) > (b.Y-a.Y) * (c.X-a.X)
        return ccw(s1[0],s2[0],s2[1]) != ccw(s1[1],s2[0],s2[1]) and ccw(s1[0],s1[1],s2[0]) != ccw(s1[0],s1[1],s2[1])

    def _rects_intersect(self, rect1, rect2):
        if self._point_in_rect(rect1.Position, rect2) or self._point_in_rect(rect2.Position, rect1):
            return True # Checks if a rect is completely inside the other one.
        for s1 in rect1.segments(): # If not, check if segments intersect.
            for s2 in rect2.segments():
                if self._segments_intersect(s1, s2):
                    return True
        return False

    def _point_in_rect(self, point, rect): # Calculs persos, a tester #PS22
        phi = math.atan2(point.Y - rect.Position.Y, point.X - rect.Position.X)
        phi += 2 * math.pi if phi < 0 else 0
        a = rect.Position.A #if rect.Position.A > 0 else rect.Position.a + 2 * math.pi
        d = math.sqrt((point.X - rect.Position.X) ** 2 + (point.Y - rect.Position.Y) ** 2)
        local_point = (d * math.cos(phi - a), d * math.sin(phi - a))
        return - rect.Width / 2.0 <= local_point[0] <= rect.Width / 2.0 and - rect.Height / 2.0 <= local_point[1] <= rect.Height / 2.0

    def _point_in_circle(self, point, circle):
        d = math.sqrt((point.X - circle.Position.X) ** 2 + (point.Y - circle.Position.Y) ** 2)
        return d <= circle.Radius

    def _rect_intersects_circle(self, rect, circle):
        new_rect = RectObstacle(rect.Position, rect.Width + circle.Radius * 2, rect.Height + circle.Radius * 2)
        return self._point_in_rect(circle.Position, new_rect)

    def _circles_intersect(self, circle1, circle2):
        d = math.sqrt((circle2.Position.X - circle1.Position.X) ** 2 + (circle2.Position.Y - circle1.Position.Y) ** 2)
        return d <= circle1.Radius + circle2.Radius
