#!/usr/bin/python

class Path(object):
    def __init__(self):
        pass

    def getWaypoints(self):
        pass

    def toShapes(self):
        pass


class NavStatus(object):
    STOPPED = 0
    MOVING_STRAIGHT = 1
    MOVING_TURNING = 2


class Robot(MapObject):
    def __init__(self, shape):
        position = None # TODO
        super(Robot, self).__init__(shape, position)
        self.NavMode = NavStatus.STOPPED
        self.CurrentPath = Path()

        self.collisions = CollisionChecker()
