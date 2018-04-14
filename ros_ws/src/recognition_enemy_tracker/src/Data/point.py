#!/usr/bin/env python
# -*-coding:Utf-8 -*

__author__ = "Gaëtan Blond"
__date__ = 10/4/2018

import math

class Point(object):
    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y

    def norm2DistTo(self, other):
        return self.norm2Dist(self, other)
    
    def __sub__(self, other): # returns a vector
        return Point(self.x - other.x, self.y - other.y)
    
    def __div__ (self, scalar):
        return Point(self.x / scalar, self.y / scalar)
    
    @staticmethod
    def norm2Dist(pointA, pointB):
        deltaX = (pointA.x - pointB.x)**2
        deltaY = (pointA.y - pointB.y)**2
        return math.sqrt(deltaX + deltaY)