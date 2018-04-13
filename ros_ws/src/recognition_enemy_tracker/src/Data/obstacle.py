#!/usr/bin/env python
# -*-coding:Utf-8 -*

__author__ = "Gaëtan Blond"
__date__ = 10/4/2018

from point import Point

class Obstacle(object):
    def __init__ (self, pos = Point(), stamp = 0):
        self.pos = pos
        self.stamp = stamp
        self._owned = False
    
    def isOwned(self):
        return self._owned
    
    def setOwned(self):
        self._owned = True