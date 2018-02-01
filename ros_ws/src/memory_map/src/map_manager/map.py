#!/usr/bin/python
import time
import rospy
from map_loader import MapLoader, LoadingHelpers
from map_bases import DictManager, RequestPath
from map_classes import Terrain, Zone, Waypoint, Entity, Container, Object
from map_teams import Team

class Map():
    Teams = []
    CurrentTeam = ''
    MapDict = None

    @staticmethod
    def load():
        starttime = time.time() * 1000
        initdict_config    = MapLoader.loadFile("1_Config.yml")["config"]
        initdict_terrain   = MapLoader.loadFile("2_Terrain.yml")["terrain"]
        initdict_zones     = MapLoader.loadFile("3_Zones.yml")["zones"]
        initdict_waypoints = MapLoader.loadFile("4_Waypoints.yml")["waypoints"]
        initdict_entities  = MapLoader.loadFile("5_Entities.yml")["entities"]
        initdict_objects   = MapLoader.loadFile("6_Objects.yml")["objects"]

        # Setting current team to the default set one.
        for team in initdict_config["teams"]:
            if bool(initdict_config["teams"][team]["default"]) is True:
                if Map.CurrentTeam != '':
                    raise ValueError("ERROR Two or more teams have been set to default.")
                Map.CurrentTeam = team
            Map.Teams.append(Team(team, initdict_config["teams"][team]))

        # Instantiate objects before creating the map dict
        for zone in initdict_zones:
            initdict_zones[zone] = Zone(initdict_zones[zone])
        for waypoint in initdict_waypoints:
            initdict_waypoints[waypoint] = Waypoint(initdict_waypoints[waypoint])
        for entity in initdict_entities:
            initdict_entities[entity] = Entity(initdict_entities[entity])
        for obj in initdict_objects:
            if "_container_" in obj:
                initdict_objects[obj] = Container(initdict_objects[obj])
            else:
                initdict_objects[obj] = Object(initdict_objects[obj])
        rospy.loginfo("Loaded files in {0:.2f}ms.".format(time.time() * 1000 - starttime))

        # Create main Map dict
        Map.MapDict = DictManager({
            "terrain":   Terrain(initdict_terrain),
            "zones":     DictManager(initdict_zones),
            "waypoints": DictManager(initdict_waypoints),
            "entities":  DictManager(initdict_entities),
            "objects":   DictManager(initdict_objects)
        })
        rospy.loginfo("Loaded map in {0:.2f}ms.".format(time.time() * 1000 - starttime))

    @staticmethod
    def swap_team(team_name):
        if team_name != Map.CurrentTeam:
            for team in Map.Teams:
                if team.name == team_name:
                    team.swap()
                    return
            rospy.logerr("Found no team with name '{}', aborting team swap.".format(team_name))

    @staticmethod
    def get(requestpath):
        if requestpath[0] != "/":
            rospy.logerr("    GET Request failed : global search needs to start with '/'.")
            return None
        return Map.MapDict.get(requestpath)

    @staticmethod
    def set(requestpath, mode, instance = None):
        if requestpath[0] != "/":
            rospy.logerr("    SET Request failed : global search needs to start with '/'.")
            return None
        return Map.MapDict.set(requestpath, mode, instance)

    @staticmethod
    def transform(codes):
        return Map.MapDict.transform(codes)
