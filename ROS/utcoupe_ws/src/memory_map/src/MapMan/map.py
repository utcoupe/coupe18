#!/usr/bin/python
import rospy
from loader import MapLoader, LoadingHelpers
from map_bases import DictManager
from map_classes import Terrain, Zones, Waypoints, Entities, Objects

class Map(DictManager):
    def __init__(self, filename):
        initdict = MapLoader.loadFile(filename)

        LoadingHelpers.checkKeysExist(initdict, "terrain", "zones", "waypoints", "entities", "objects")
        super(Map, self).__init__({
            "terrain": Terrain(initdict["terrain"]),
            "zones": Zones(initdict["zones"]),
            "waypoints": Waypoints(initdict["waypoints"]),
            "entities": Entities(initdict["entities"]),
            "objects": Objects(initdict["objects"])
        })
