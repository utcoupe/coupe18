#!/usr/bin/python
import rospy
from collisions_attributes import Shape, Position, Velocity




class MapObject(object):
    def __init__(self, shape, position, velocity = None):
        self.Position = position
        self.Shape = shape


class Obstacle(MapObject):
    def __init__(self, shape, position, velocity = None):
        super(Obstacle, self).__init__(shape, position, velocity)







class Enemy(Robot):
    pass # Used just to differentiate the names.


class CollisionLevel(object):
    LEVEL_STOP = 0      # Collision will happen very soon, stop the robot!
    LEVEL_DANGER = 1    # Path needs to be modified or a collision will happen.
    LEVEL_POTENTIAL = 2 # A dynamic object will probably pass in some time, maybe not (replan optional).

class Collision(object):
    def __init__(self, obstacle, level):
        pass
