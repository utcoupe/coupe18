#!/usr/bin/python
import math
import rospy
from map_shapes import Rect, Circle
from map_classes import Position, MapObstacle, RectObstacle, Velocity
from robot_path import RobotPath
from collisions import Collision, CollisionLevel, CollisionsResolver, CollisionThresholds


class RobotStatus(object): # Status given by navigation/nagivator.
    NAV_IDLE     = 0 # No movement planned.
    NAV_STOPPED  = 1 # Robot stopped, but still has movement plans pending.
    NAV_STRAIGHT = 2 # Robot moving in a straight line.
    NAV_TURNING  = 3 # Robot turning around itself.


class MapRobot(MapObstacle):
    def __init__(self, shape):
        super(MapRobot, self).__init__(shape, None, Velocity(0, 0)) # TODO velocicty
        self.NavStatus = RobotStatus.NAV_IDLE
        self.Path = RobotPath()
        self._engine = CollisionsResolver()

    def isInitialized(self):
        return self.Shape != None and self.Position != None and self.Velocity != None # TODO implement check if true data in there

    def updatePosition(self, new_position):
        self.Position = new_position

    def updatePath(self, new_path):
        self.Path = new_path

    def updateVelocity(self, linear, angular):
        self.Velocity.Linear = linear
        self.Velocity.Angular = angular

    def getStopRect(self):
        if not self.Path.hasPath():
            return []
        r = Rect(self.Shape.Height + CollisionThresholds.STOP_DISTANCE, self.Shape.Width)
        l = r.Width / 2.0 - self.Shape.Height / 2.0
        return RectObstacle(r, Position(self.Position.X + l * math.cos(self.Position.A),
                                        self.Position.Y + l * math.sin(self.Position.A),
                                        self.Position.A)) # TODO support circles

    def checkCollisions(self, map_obstacles):
        if map_obstacles is not None and self.Path.hasPath() and self.isInitialized():
            if self.NavStatus != RobotStatus.NAV_IDLE:
                rospy.logdebug("Checking for collisions...")
                imminent_collisions = self._check_imminent_collisions(map_obstacles)
                path_collisions = self._check_path_collisions(map_obstacles)
                for pc in path_collisions:
                    if pc.Obstacle not in [ic.Obstacle for ic in imminent_collisions]:
                        imminent_collisions.append(pc)
                return imminent_collisions
        else:
            return []

    def _check_path_collisions(self, map_obstacles):
        return self.Path.checkCollisions(self, map_obstacles)

    def _check_imminent_collisions(self, map_obstacles):
        collisions = []
        danger_rect = self.getStopRect()
        for obs in map_obstacles:
            if self._engine.intersect(danger_rect, obs):
                collisions.append(Collision(CollisionLevel.LEVEL_STOP, obs, 0.0))
        return collisions
