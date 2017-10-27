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
        keys = Map.parsePath(path)
        restricted_dict = Map.nested_get(Map.MapDict, keys, new_value)
        print restricted_dict
        restricted_dict = new_value
        print restricted_dict
        print Map.nested_get(Map.MapDict, keys, new_value)
        return 200, "TODO"


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
    def nested_get(mapdict, keys, new_value):
        if len(keys) > 1:
            if keys[0] in mapdict:
                return Map.nested_get(mapdict[keys[0]], keys[1:], new_value)
            else:
                return None
        elif len(keys) == 1:
            return mapdict[keys[0]]
        '''
        mapdict = Map.MapDict
        keys = Map.parsePath(path)

        for k in keys:
            print str(mapdict) + "\n\n"
            mapdict = mapdict.setdefault(k, None)
            if mapdict is None:
                rospy.logerr("Path key not found in dict. Invalid path.")
                return 404, "Key '{}' in path not found in the map dict.".format(k)

        mapdict = new_value
        return 200, ""
        '''

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
