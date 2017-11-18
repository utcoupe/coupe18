#!/usr/bin/python

class Velocity(object):
    pass


class Position(object):
    pass


class Shape(object):
    def __init__(self):
        pass

class Rect(Shape):
    def __init__(self, width, height):
        super(Rect, self).__init__()
        self.Width = width
        self.Height = height

class Circle(Shape):
    pass

class Point(Shape):
    pass


class MapObstacle(object):
    def __init__(self, shape, position, velocity = None):
        self.Position = Position()
        self.Shape = Shape()
        self.Velocity = Velocity()
