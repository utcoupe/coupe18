#!/usr/bin/python
import math
from map_shapes import Shape, Rect, Circle
from map import Map


class Velocity(object):
    pass # TODO


class Point(object):
    def __init__(self, x, y):
        self.X = x
        self.Y = y

class Position(Point):
    def __init__(self, x, y, angle = 0.0):
        super(Position, self).__init__(x, y)
        self.A = angle

    def tuple2(self):
        return (self.X, self.Y)


class MapObstacle(object):
    def __init__(self, shape, position, velocity = None):
        self.Position = position
        self.Shape = shape
        self.Velocity = velocity

    def intersects(self, obstacle):
        raise NotImplementedError

class CircleObstacle(MapObstacle):
    def __init__(self, shape, position):
        super(CircleObstacle, self).__init__(shape, position)

    def toEctagon(self):
        return NotImplementedError

    def distanceToCollision(self, obstacle):
        return 0.35 # TODO Not implemented

    def intersects(self, obstacle):
        return False

    def __repr__(self):
        return "circle"


class RectObstacle(MapObstacle):
    def __init__(self, shape, position):
        super(RectObstacle, self).__init__(shape, position)

    def corners(self): # Calculs persos, a verifier DEPRECATED
        corners = []
        l = math.sqrt((self.Shape.Width / 2.0) ** 2 + (self.Shape.Height / 2.0) ** 2)
        corner_phi = math.atan2(self.Shape.Height , self.Shape.Width)
        for angle_phi in [0, math.pi]:
            for i in range(2):
                phi = angle_phi  + ((-1) ** (i + 1)) * corner_phi
                corners.append(Point(self.Position.X + l * math.cos(phi + self.Position.A), self.Position.Y + l * math.sin(phi + self.Position.A)))

        for c in corners: # TODO Remove
            Map.LidarObjects.append(MapObstacle(Circle(0.02), Position(c.X, c.Y)))
        return corners

    def segments(self):
        c = self.corners()
        return [(c[i], c[(i + 1) % 4]) for i in range(len(c))]

    def distanceToCollision(self, obstacle):
        return 0.6 # TODO NotImplemented

    def __repr__(self):
        return "rect"
