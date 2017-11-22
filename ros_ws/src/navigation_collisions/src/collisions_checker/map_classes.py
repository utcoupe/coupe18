#!/usr/bin/python

class Velocity(object):
    pass


class Position(object):
    def __init__(self, x, y, angle = 0.0):
        self.X = x
        self.Y = y
        self.A = angle


class Shape(object):
    def __init__(self):
        pass

class Rect(Shape):
    def __init__(self, width, height):
        super(Rect, self).__init__()
        self.Width = width
        self.Height = height

class Circle(Shape):
    def __init__(self, radius):
        super(Circle, self).__init__()
        self.Radius = radius

class Point(Shape):
    pass


class MapObstacle(object):
    def __init__(self, shape, position, velocity = None):
        self.Position = position
        self.Shape = shape
        self.Velocity = velocity

    def intersects(self, obstacle):
        return True  # TODO Not implemented, returns true for now
    def distanceToCollision(self, obstacle):
        return 0.2 # TODO Not implemented
