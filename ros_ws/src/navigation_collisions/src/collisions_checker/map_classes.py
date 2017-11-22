#!/usr/bin/python


class Velocity(object):
    pass # TODO


class Position(object):
    def __init__(self, x, y, angle = 0.0):
        self.X = x
        self.Y = y
        self.A = angle

    def tuple2(self):
        return (self.X, self.Y)


class MapObstacle(object):
    def __init__(self, shape, position, velocity = None):
        self.Position = position
        self.Shape = shape
        self.Velocity = velocity

    def intersects(self, obstacle):
        return self.Shape.intersects(self.Position, obstacle)
