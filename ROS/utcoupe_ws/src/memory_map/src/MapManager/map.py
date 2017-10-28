#!/usr/bin/python
import rospy
from loader import MapLoader, LoadingHelpers
from map_bases import MapElement, ListManager
import map_classes


class Map(MapElement):
    Terrain = None
    Zones = None
    Waypoints = None
    Entities = None
    Objects = None

    @staticmethod
    def load(filename):
        #super(Map, self).__init__("map")
        initdict = MapLoader.loadFile(filename)

        # Loading objects from the YML dict
        LoadingHelpers.checkKeysExist(initdict, "terrain", "zones", "waypoints", "entities", "objects")
        Map.Terrain    = map_classes.Terrain(initdict["terrain"])
        Map.Zones      = ListManager("zones", map_classes.Zone, initdict["zones"])
        Map.Waypoints   = ListManager("waypoints", map_classes.Waypoint, initdict["waypoints"])

        Map.Entities   = ListManager("entities", map_classes.Entity, initdict["entities"])
        Map.Objects    = map_classes.ObjectContainer("objects", initdict) # Small hack to properly initialize ObjectContainer
        rospy.loginfo("[memory/map] Loaded map successfully.")

    @staticmethod
    def get(mappath):
        key = mappath.getNextKey()
        if key.Extension == "list":
            if key.KeyName == Map.Terrain.ClassType:
                return Map.Terrain.get(mappath)
            elif key.KeyName == Map.Zones.ClassType:
                return Map.Zones.get(mappath)
            elif key.KeyName == Map.Waypoints.ClassType:
                return Map.Waypoints.get(mappath)
            elif key.KeyName == Map.Entities.ClassType:
                return Map.Entities.get(mappath)
            elif key.KeyName == Map.Objects.ClassType:
                return Map.Objects.get(mappath)
            else:
                rospy.logerr("[memory/map] GET request: Map couldn't find list named '{}'".format(key))
        else:
            rospy.logerr("[memory/map] GET request: Map got unrecognized path key extension '{}'".format(key))
        rospy.logwarn("got get request, path is " + str(mappath.Keys))

    @staticmethod
    def set(mappath, new_value):
        rospy.logwarn("got set request")
