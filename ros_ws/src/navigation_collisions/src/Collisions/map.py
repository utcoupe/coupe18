#!/usr/bin/python

class MapObstacles(object):
    Enemies = []
    BeltPoints = []
    LidarObjects = []

    @staticmethod
    def toList():
        return MapObstacles.Enemies + MapObstacles.BeltPoints + MapObstacles.LidarObjects

    @staticmethod
    def hasData():
        return MapObstacles.Enemies or MapObstacles.BeltPoints or MapObstacles.LidarObjects
