#!/usr/bin/python2.7
# -*-coding:Utf-8 -*

class Rect:
    def __init__(self,x=0,y=0,w=0,h=0):
        self.x = x
        self.y = y
        self.w = w
        self.h = h
    
    def __repr__(self):
        return 'Rect( x={}, y={}, w={}, h={} )'.format(self.x, self.y, self.w, self.h)
