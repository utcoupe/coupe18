#!/usr/bin/python
import rospy
from map_loader import MapLoader
from functools import reduce  # forward compatibility for Python 3
import operator

def getFromDict(dataDict, mapList):
    return reduce(operator.getitem, mapList, dataDict)

class Map():
    MapDict = {}

    @staticmethod
    def load(filename):
        Map.MapDict = MapLoader().load(filename)

    @staticmethod
    def get(path):
        return Map.findFromPath(path)

    @staticmethod
    def set(path, new_value):
        return Map.nested_set(path, new_value)

    @staticmethod
    def findFromPath(path):
        dataDict = Map.MapDict
        keys = Map.parsePath(path)
        for k in keys: 
            if k in dataDict:
                dataDict = dataDict[k]
            else: return None
        return dataDict

    @staticmethod
    def nested_set(path, new_value):
        mapdict = Map.MapDict
        keys = Map.parsePath(path)

        for k in keys:
            print str(mapdict) + "\n\n"
            mapdict = mapdict.setdefault(k, None)
            if mapdict == None:
                rospy.logerr("Path key not found in dict. Invalid path.")
                return 404, "Key '{}' in set path not found in the map dict.".format(k)

        mapdict = new_value
        return 200, ""

    @staticmethod
    def parsePath(path):
        p = [p for p in path.split("/") if p != ""]
        rospy.logdebug("Parsed path to " + str(p))
        return p
    '''
    @staticmethod
    def findFromPath(path):
        filters = path.split("/")
        return reduce(operator.getitem, filters, Map.MapDict)
        
        result = Map.MapDict # TODO: remove ?
        for f in filters[1:]:
            result = result[f]
            #print "Applied filter {} : {}".format(f, result)
        return result
    '''
    @staticmethod
    def getMapDict():
        return Map.MapDict

    @staticmethod
    def getElement(objectname):  # TODO:
        for elem in Map.MapDict["objects"]:
            o = elem.getObject(objectname)
            if o: 
                return o
        return None
