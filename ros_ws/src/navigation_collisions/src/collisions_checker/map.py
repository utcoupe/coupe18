#!/usr/bin/python


class Map(object):
    Robot = None

    Enemies = []
    BeltPoints = []
    LidarObjects = []

    @staticmethod
    def toList():
        return Map.Enemies + Map.BeltPoints + Map.LidarObjects

    @staticmethod
    def hasData():
        return Map.Enemies or Map.BeltPoints or Map.LidarObjects
