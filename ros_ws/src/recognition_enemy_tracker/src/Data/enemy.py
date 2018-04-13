#!/usr/bin/env python
# -*-coding:Utf-8 -*

__author__ = "Gaëtan Blond"
__date__ = 10/4/2018

import rospy
from point import Point
from obstacle import Obstacle
from recognition_enemy_tracker.msg import EnemyStamped

class Enemy(object):
    def __init__ (self):
        self._lastSeen = rospy.Time(0)
        self._lastPos = Point()
        self._speed = Point()
        self._owner = False

    def updatePos(self, obstacle):
        self._owner = True
        
        if self._lastSeen == obstacle.stamp:
            return
        
        if self._lastSeen != rospy.Time(0):
            timeElapsed = obstacle.stamp - self._lastSeen
            vect = obstacle.pos - self._lastPos
            self._speed = vect/timeElapsed.to_sec()

        self._lastSeen = obstacle.stamp
        self._lastPos = obstacle.pos
    
    def isOwner(self):
        return self._owner
    
    def releaseOwnership(self):
        self._owner = False
    
    def getPos(self):
        return self._lastPos
    
    def toEnemyStamped(self):
        es = EnemyStamped()
        es.pos.x = self._lastPos.x
        es.pos.y = self._lastPos.y
        es.speed.x = self._speed.x
        es.speed.y = self._speed.y
        # es.header.stamp = self._lastSeen
        return es
        