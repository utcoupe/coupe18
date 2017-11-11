#!/usr/bin/python
import rospy


class Path(object):
    def __init__(self):
        pass


class Obstacle(object):
    def __init__(self, position, shape):
        self.Position = position
        self.Shape = shape


class StaticObject(Obstacle):
    def __init__(self, position, shape):
        super(StaticObject, self).__init__(position, shape)


class DynamicObject(Obstacle):
    def __init__(self, position, shape, velocity):
        super()
        self.Velocity


class Robot(DynamicObject):
    def __init__(self, shape, position):
        self.Shape = None
        self.Position = None
        self.Velocity = None
