#!/usr/bin/python2.7
# -*-coding:Utf-8 -*
from numpy import array
from numpy.linalg import solve
from math import sqrt

class Rect:
    def __init__(self,x=0,y=0,w=0,h=0,t=0):
        self.x = x
        self.y = y
        self.w = w
        self.h = h
        self.t = t

    def toXPoint(self):
        return Point(self.t, self.x)

    def toYPoint(self):
        return Point(self.t, self.y)

    def diagonal(self):
        return sqrt(self.w**2+self.h**2)

    def __repr__(self):
        return 'Rect( x={}, y={}, w={}, h={}, t={} )'.format(self.x, self.y, self.w, self.h, self.t)
class Point:
    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y

    def __repr__(self):
        return '({}, {})'.format(self.x, self.y)
        