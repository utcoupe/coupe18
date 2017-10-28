#!/usr/bin/python
import rospy
from loader import MapLoader, LoadingHelpers
from map_bases import MapElement, ListManager
import map_classes


class Map(MapElement):
    Terrain = None
    Entities = None
    Objects = None

    @staticmethod
    def load(filename):
        #super(Map, self).__init__("map")
        initdict = MapLoader.loadFile(filename)

        # Loading objects from the YML dict
        LoadingHelpers.checkKeysExist(initdict, "terrain", "entities", "objects")
        Map.Terrain    = map_classes.Terrain("terrain", initdict["terrain"])
        Map.Entities   = ListManager("entities", map_classes.Entity, initdict["entities"])
        Map.Objects    = map_classes.ObjectContainer("objects", initdict) # Small hack to properly initialize ObjectContainer
        rospy.loginfo("[memory/map] Loaded map successfully.")

    @staticmethod
    def get(mappath):
        rospy.logwarn("got get request")

    @staticmethod
    def set(mappath, new_value):
        rospy.logwarn("got set request")
