#!/usr/bin/python
import rospy
from collisions_attributes import Shape, Position, Velocity


class Path(object):
    def __init__(self):
        pass


class Obstacle(object):
    def __init__(self, shape, position):
        self.Position = position
        self.Shape = shape


class StaticObject(Obstacle):
    def __init__(self, shape, position):
        super(StaticObject, self).__init__(shape, position)


class DynamicObject(Obstacle):
    def __init__(self, shape, position, velocity):
        super(DynamicObject, self).__init__(shape, position)
        self.Velocity = velocity


class Robot(DynamicObject):
    def __init__(self, shape):
        position = None # TODO
        velocity = None
        super(Robot, self).__init__(shape, position, velocity)
        self.Navigating = False
        self.CurrentPath = Path()


class BeltPoints(object):
    def __init__(self):
        self.TerrainPoints = []
        self.ObstaclePoints = []
