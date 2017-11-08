#!/usr/bin/python
import rospy
from map_loader import MapLoader, LoadingHelpers
from map_bases import DictManager, ListManager
from map_classes import Terrain, Zone, Waypoint, Entity, Object

class Map(DictManager):
    def __init__(self, filename):
        initdict = MapLoader.loadFile(filename)

        LoadingHelpers.checkKeysExist(initdict, "terrain", "zones", "waypoints", "entities", "objects")
        super(Map, self).__init__({
            "terrain": Terrain(initdict["terrain"]),
            "zones": ListManager(Zone, initdict["zones"]),
            "waypoints": ListManager(Waypoint, initdict["waypoints"]),
            "entities": ListManager(Entity, initdict["entities"]),
            "objects": ListManager(Object, initdict["objects"])
        })
