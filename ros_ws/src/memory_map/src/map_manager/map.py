#!/usr/bin/python
import time
import rospy
from map_loader import MapLoader, LoadingHelpers
from map_bases import DictManager, ListManager, RequestPath
from map_classes import Terrain, Zone, Waypoint, Entity, Object

class Map():
    MapDict = None

    @staticmethod
    def load():
        starttime = time.time() * 1000
        initdict_terrain   = MapLoader.loadFile("../../def/1_Terrain.yml")["terrain"]
        initdict_zones     = MapLoader.loadFile("../../def/2_Zones.yml")["zones"]
        initdict_waypoints = MapLoader.loadFile("../../def/3_Waypoints.yml")["waypoints"]
        initdict_entities  = MapLoader.loadFile("../../def/4_Entities.yml")["entities"]
        initdict_objects   = MapLoader.loadFile("../../def/5_Objects.yml")["objects"]
        # Instantiate objects before creating the map dict
        for zone in initdict_zones:
            initdict_zones[zone] = Zone(initdict_zones[zone])
        for waypoint in initdict_waypoints:
            initdict_waypoints[waypoint] = Waypoint(initdict_waypoints[waypoint])
        for entity in initdict_entities:
            initdict_entities[entity] = Entity(initdict_entities[entity])
        for obj in initdict_objects:
            initdict_objects[obj] = Object(initdict_objects[obj])
        rospy.loginfo("Loaded files in {0:.2f}ms.".format(time.time() * 1000 - starttime))

        # Create main Map dict
        Map.MapDict = DictManager({
            "terrain": Terrain(initdict_terrain),
            "zones": DictManager(initdict_zones),
            "waypoints": DictManager(initdict_waypoints),
            "entities": DictManager(initdict_entities),
            "objects": DictManager(initdict_objects)
        })
        rospy.loginfo("Loaded map in {0:.2f}ms.".format(time.time() * 1000 - starttime))

    @staticmethod
    def get(requestpath):
        if requestpath[0] != "/":
            rospy.logerr("    GET Request failed : global search needs to start with '/'.")
            return None
        return Map.MapDict.get(requestpath)

    @staticmethod
    def set(requestpath):
        if requestpath[0] != "/":
            rospy.logerr("    GET Request failed : global search needs to start with '/'.")
            return None
        return Map.MapDict.set(requestpath)
