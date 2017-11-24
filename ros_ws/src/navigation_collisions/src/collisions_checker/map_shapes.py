#!/usr/bin/python


class Shape(object):
    pass


class Rect(Shape):
    def __init__(self, width, height):
        super(Rect, self).__init__()
        self.Width = width
        self.Height = height

    def __repr__(self):
        return "rect"

class Circle(Shape):
    def __init__(self, radius):
        super(Circle, self).__init__()
        self.Radius = radius

    def __repr__(self):
        return "circle"
