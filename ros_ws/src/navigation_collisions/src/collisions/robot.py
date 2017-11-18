#!/usr/bin/python
from map_classes import MapObstacle
from robot_path import RobotPath


class RobotStatus(object): # Status given by navigation/nagivator.
    NAV_IDLE     = 0 # No movement planned.
    NAV_STOPPED  = 1 # Robot stopped, but still has movement plans pending.
    NAV_STRAIGHT = 2 # Robot moving in a straight line.
    NAV_TURNING  = 3 # Robot turning around itself.


class MapRobot(MapObstacle):
    def __init__(self, shape):
        super(MapRobot, self).__init__(shape, None, None)
        self.NavStatus = RobotStatus.NAV_STOPPED
        self.Path = RobotPath()

    def isInitialized(self):
        return self.Shape != None and self.Position != None and self.Velocity != None # TODO implement check if true data in there

    def updatePosition(self, new_position):
        self.Position = new_position

    def updatePath(self, new_path):
        self.Path = new_path

    def updateVelocity(self, new_velocity):
        self.Velocity = new_velocity

    def checkPathCollisions(self, map_obstacles):
        collisions = []
        if map_obstacles.hasData() and self.Path.hasPath() and self.isInitialized():
            if self.NavStatus != RobotStatus.NAV_IDLE:
                collisions = self.Path.checkCollisions(map_obstacles)

        return collisions
