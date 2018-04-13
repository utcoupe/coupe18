#!/usr/bin/env python
# -*-coding:Utf-8 -*

__author__ = "Gaëtan Blond"
__date__ = 10/4/2018

import rospy
from Data import Enemy, Point, Obstacle

from recognition_objects_classifier.msg import ClassifiedObjects
from ai_game_status import StatusServices

NODE_NAME = "recognition"
NODE_NAMESPACE = "enemy_tracker"
FULL_NODE_NAME = "/" + NODE_NAMESPACE + "/" + NODE_NAME

NB_ENEMIES = 3
PUB_ENEMIES_POS_RATE = 10 # Hz

CLASSIFIED_OBSTACLES_TOPIC = "/recognition/objects_classifier/objects"

class EnemyTrackerNode(object):
    def __init__ (self):
        self._lastObstacles = []
        self._enemies = []
        for idEnemy in range(1, NB_ENEMIES):
            self._enemies.append(Enemy())
        
        rospy.init_node(NODE_NAME)
        self._obsTaclesSubscriber = rospy.Subscriber(CLASSIFIED_OBSTACLES_TOPIC, ClassifiedObjects, self._classifiedObjectsCallback)
        rospy.loginfo("enemy_tracker up and tracking.")
        StatusServices(NODE_NAME, NODE_NAMESPACE).ready(True)
        self.run()
    
    def run(self):
        rate = rospy.Rate(PUB_ENEMIES_POS_RATE)
        # TODO mutex on _lastObstacles ?
        while not rospy.is_shutdown():
            self._processObstacles()
            self._publishEnemyPoses()
            rate.sleep()
    
    def _processObstacles(self):
        sortedObstacleIdsPerEnemies = []

        for enemy in self._enemies:
            enemy.releaseOwnership()
            sortedObstacles = self._sortObstacles(enemy)
            sortedObstacleIdsPerEnemies.append(sortedObstacles)
        
        # TODO store next not owned obstacle for each list
        for idAffectation in range(1, len(self._enemies)):
            self._affecteObstacle(sortedObstacleIdsPerEnemies)
    
    def _sortObstacles (self, enemy):
        obstacleIds = range(1, len(self._lastObstacles))
        obstacleIds = sorted(obstacleIds, key=lambda obsId : Point.norm2Dist(self._lastObstacles[obsId].pos, enemy.getPos()))
        return obstacleIds
    
    def _affecteObstacle (self, sortedObstacleIdsPerEnemies):
        bestEnemyId = 0
        bestObsId = 0
        bestDist = 0

        for enemyId in range(1, len(self._enemies)):
            if self._enemies[enemyId].isOwner():
                continue
            obsId = self._findNextNotOwnedObstacle(sortedObstacleIdsPerEnemies[enemyId])
            if obsId == 0: # all obstacles already affected or no obstacles known
                break
            dist = Point.norm2Dist(self._enemies[enemyId].getPos(), self._lastObstacles[obsId].pos)
            if dist > bestDist:
                bestDist = dist
                bestObsId = obsId
                bestEnemyId = enemyId

        if bestEnemyId != 0 and bestObsId != 0:
            self._enemies[bestEnemyId].updatePos(self._lastObstacles[bestObsId])
            self._lastObstacles[bestObsId].setOwned()
    
    def _findNextNotOwnedObstacle (self, obstacleIds):
        for obsId in obstacleIds:
            if not self._lastObstacles[obsId].isOwned():
                return obsId
        return 0
    
    def _classifiedObjectsCallback (self, obstacles):
        del self._lastObstacles[:]
        for rect in obstacles.unknown_rects:
            obs = Obstacle()
            obs.pos = Point(rect.x, rect.y)
            obs.stamp = rect.header.stamp
            self._lastObstacles.append(obs)
        for circ in obstacles.unknown_circles:
            obs = Obstacle()
            obs.pos = Point(circ.circle.center.x, circ.circle.center.y)
            obs.stamp = circ.header.stamp
            self._lastObstacles.append(obs)
    
    def _publishEnemyPoses(self):
        pass


if __name__ == "__main__":
    enemy_tracker = EnemyTrackerNode()