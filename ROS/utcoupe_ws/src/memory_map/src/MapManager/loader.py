#!/usr/bin/python
import os
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

    def loafYamlFromDescriptionModule(self, name):
        pass  # TODO use this when Definitions package is ready instead of loading from file.

    def init_dict(self, data):
        MapDict = data
        # -------- Terrain

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
        for obj in objects:
            objects[obj] = Object(obj, objects[obj])

        return MapDict
