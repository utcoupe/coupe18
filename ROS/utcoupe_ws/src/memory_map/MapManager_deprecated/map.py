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
        if key.KeyName == Map.Terrain.ClassName:
            return Map.Terrain.get(mappath)
        if key.Extension == "list":
            if key.KeyName == Map.Zones.ClassName:
                return Map.Zones.get(mappath)
            elif key.KeyName == Map.Waypoints.ClassName:
                return Map.Waypoints.get(mappath)
            elif key.KeyName == Map.Entities.ClassName:
                return Map.Entities.get(mappath)
            elif key.KeyName == Map.Objects.ClassName:
                return Map.Objects.get(mappath)
            else:
                rospy.logerr("[memory/map] GET request: Map couldn't find list named '{}'".format(key))
        else:
            rospy.logerr("[memory/map] GET request: Map got unrecognized path key extension '{}'".format(key))

    @staticmethod
    def set(mappath, new_value_dict):
        key = mappath.getNextKey()
        if key.KeyName == Map.Terrain.ClassName:
            return Map.Terrain.set(mappath, new_value_dict)
        if key.Extension == "list":
            if key.KeyName == Map.Zones.ClassName:
                return Map.Zones.set(mappath, new_value_dict)
            elif key.KeyName == Map.Waypoints.ClassName:
                return Map.Waypoints.set(mappath, new_value_dict)
            elif key.KeyName == Map.Entities.ClassName:
                return Map.Entities.set(mappath, new_value_dict)
            elif key.KeyName == Map.Objects.ClassName:
                return Map.Objects.set(mappath, new_value_dict)
            else:
                rospy.logerr("[memory/map] GET request: Map couldn't find list named '{}'".format(key))
        else:
            rospy.logerr("[memory/map] GET request: Map got unrecognized path key extension '{}'".format(key))
