#!/usr/bin/python
import rospy
from loader import MapLoader, LoadingHelpers
from map_classes import Terrain, ListManager
import map_objects

class Map():
    def __init__(self, filename):
        self.load(MapLoader.loadFile(filename))

    def load(self, initdict):
        LoadingHelpers.checkKeysExist(initdict, "terrain", "entities", "objects")
        self.Terrain    = Terrain(    "terrain",                               initdict["terrain"] )
        self.Entities   = ListManager("entities", map_objects.Entity,          initdict["entities"])
        self.Objects    = ListManager("objects",  map_objects.ObjectContainer, initdict["objects"] )
        rospy.logdebug("[memory/map] Loaded map successfully.")

    def get(self, path):
        pass

    def set(self, path, new_value):
        pass





# class Map():
#     MapDict = {}

#     @staticmethod
#     def load(filename):
#         Map.MapDict = MapLoader().load(filename)

#     @staticmethod
#     def get(path):
#         return Map.findFromPath(path)

#     @staticmethod
#     def set(filter, value):
#         pass

#     @staticmethod
#     def findFromPath(path):
#         filters = path.split("/")
#         instance = Map.MapDict
#         for f in filters[1:]:
#             instance = instance[f]
#             print "Applied filter {} : {}".format(f, instance)

#     @staticmethod
#     def getMapDict():
#         return Map.MapDict

#     @staticmethod
#     def getObject(objectname):  # TODO:
#         for elem in Map.MapDict["objects"]:
#             o = elem.getObject(objectname)
#             if o: 
#                 return o
#         return None
