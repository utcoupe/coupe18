#!/usr/bin/python
import math
import rospy
from map_classes import Position, MapObstacle, RectObstacle, Velocity
from robot_path import RobotPath
from collisions import Collision, CollisionLevel, CollisionsResolver, CollisionThresholds


class RobotStatus(object): # Status given by navigation/nagivator.
    NAV_IDLE       = 0 # No movement planned.
    NAV_NAVIGATING = 1 # Pending path active.


class MapRobot(RectObstacle):
    def __init__(self, width, height):
        super(MapRobot, self).__init__(None, width, height, Velocity(0, 0)) # TODO velocicty
        self.NavStatus = RobotStatus.NAV_IDLE
        self.Path = RobotPath()
        self._engine = CollisionsResolver()

    def isInitialized(self):
        return self.Position != None and self.Velocity != None # TODO implement check if true data in there

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
        w, h = self.Height + (CollisionThresholds.STOP_DISTANCE if self.Velocity.Linear != 0 else 0), self.Width
        l = w / 2.0 - self.Height / 2.0
        side_a = math.pi if self.Velocity.Linear < 0 else 0
        return RectObstacle(Position(self.Position.X + l * math.cos(self.Position.A + side_a),
                                     self.Position.Y + l * math.sin(self.Position.A + side_a),
                                     self.Position.A), w, h)

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
