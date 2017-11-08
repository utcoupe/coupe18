#!/usr/bin/python
import time
import rospy
from map_loader import MapLoader, LoadingHelpers
from map_bases import DictManager, ListManager, RequestPath
from map_classes import Terrain, Zone, Waypoint, Entity, Object

class Map():
    MapDict = None

    @staticmethod
    def load(filename):
        starttime = time.time() * 1000
        initdict = MapLoader.loadFile(filename)
        LoadingHelpers.checkKeysExist(initdict, "terrain", "zones", "waypoints", "entities", "objects")

        # Instantiate objects before creating the map dict
        for zone in initdict["zones"]:
            initdict["zones"][zone] = Zone(initdict["zones"][zone])
        for waypoint in initdict["waypoints"]:
            initdict["waypoints"][waypoint] = Waypoint(initdict["waypoints"][waypoint])
        for entity in initdict["entities"]:
            initdict["entities"][entity] = Entity(initdict["entities"][entity])
        for obj in initdict["objects"]:
            initdict["objects"][obj] = Object(initdict["objects"][obj])

        # Create main Map dict
        Map.MapDict = DictManager({
            "terrain": Terrain(initdict["terrain"]),
            "zones": DictManager(initdict["zones"]),
            "waypoints": DictManager(initdict["waypoints"]),
            "entities": DictManager(initdict["entities"]),
            "objects": DictManager(initdict["objects"])
        })
        rospy.loginfo("Loaded map in {0:.2f} ms.".format(time.time() * 1000 - starttime))

    @staticmethod
    def get(requestpath):
        return Map.MapDict.get(RequestPath(requestpath))

    @staticmethod
    def set(requestpath, new_value):
        return Map.MapDict.set(requestpath, new_value)
