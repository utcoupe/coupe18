#!/usr/bin/python
import rospy, os
from objects import *


class MapLoader():
    def load(self, filename):
        return self.init_dict(self.loadYamlFromFile(filename))

    def loadYamlFromFile(self, filename):
        import yaml
        with open(os.path.dirname(__file__) + "/" + filename, 'r') as stream:
            try:
                return yaml.load(stream)
            except yaml.YAMLError as exc:
                print exc

    def loafYamlFromDescriptionModule(self, path):
        # TODO use this when Definitions package is ready instead of loading from file.
        pass

    def init_dict(self, data):
        MapDict = data
        # -------- Terrain
        zones = MapDict["terrain"]["zones"]
        for zone in zones:
            zones[zone] = Zone(zone, zones[zone])

        # -------- Waypoints
        waypoints = MapDict["terrain"]["waypoints"]
        for waypoint in waypoints:
            waypoints[waypoint] = Waypoint(waypoint, waypoints[waypoint])

        # -------- Entities
        entities = MapDict["entities"]
        for entity in entities:
            entities[entity] = Entity(entity, entities[entity])

        # -------- Objects
        objects = MapDict["objects"]
        for elem in objects:
            #if objects[elem]["type"] == "namespace":
            #    objects[elem] = ObjectNamespace(elem, objects[elem])
            #elif objects[elem]["type"] == "container":
            #    rospy.logerr("Object '{}' found in the map description file that doesn't belong to any namespace. object not loaded".format(elem))
            if objects[elem]["type"] == "object":
                objects[elem] = Object(elem, objects[elem])
                #rospy.logerr("Object '{}' found in the map description file that doesn't belong to any namespace. object not loaded".format(elem))

        return MapDict
