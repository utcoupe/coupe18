#!/usr/bin/python
import rospy


class Shape(object):
    def __init__(self):
        self.Type = None

    def intersects(self, obstacle):
        return True


class PathShape(Shape):
    


class Position(object):
    def __init__(self):
        pass


class Velocity(object):
    def __init__(self):
        pass
