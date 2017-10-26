#!/usr/bin/python
import rospy
from map_loader import MapLoader


class Map():
    MapDict = {}

    @staticmethod
    def load(filename):
        Map.MapDict = MapLoader().load(filename)

    @staticmethod
    def get(path):
        return Map.findFromPath(path)

    @staticmethod
    def set(filter, value):
        pass

    @staticmethod
    def findFromPath(path):
        filters = path.split("/")
        result = Map.MapDict
        for f in filters[1:]:
            result = result[f]
            #print "Applied filter {} : {}".format(f, result)
        return result

    @staticmethod
    def getMapDict():
        return Map.MapDict

    @staticmethod
    def getObject(objectname):  # TODO:
        for elem in Map.MapDict["objects"]:
            o = elem.getObject(objectname)
            if o: 
                return o
        return None
