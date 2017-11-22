#!/usr/bin/python


class Shape(object):
    def intersects(self, self_position, obstacle):
        raise NotImplementedError  # Must be overwritten


class Rect(Shape):
    def __init__(self, width, height):
        super(Rect, self).__init__()
        self.Width = width
        self.Height = height

    def intersects(self, self_position, obstacle):
        if isinstance(obstacle, Rect):
            pass
        elif isinstance(obstacle, Circle):
            pass
        return True  # TODO


class Circle(Shape):
    def __init__(self, radius):
        super(Circle, self).__init__()
        self.Radius = radius

    def intersects(self, self_position, obstacle):
        if isinstance(obstacle, Rect):
            pass
        elif isinstance(obstacle, Circle):
            pass
        return True # TODO
