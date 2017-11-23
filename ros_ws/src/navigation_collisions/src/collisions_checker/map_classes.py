#!/usr/bin/python


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
