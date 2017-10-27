#!/usr/bin/python
from loader import LoadingHelpers
from map_bases import MapElement, ListManager
from map_objects import *


'''
MAP METACLASSES
'''

class Terrain(MapElement):
    def __init__(self, name, initdict):
        super(Terrain, self).__init__(name)
        LoadingHelpers.checkKeysExist(initdict, "walls", "zones", "waypoints")
        #self.Walls = ListManager("zones", map_objects.Zone, initdict["zones"])  # TODO: implement
        self.Zones = ListManager("zones", Zone, initdict["zones"])
        self.Waypoints = ListManager("waypoints", Waypoint, initdict["waypoints"])

        LoadingHelpers.checkKeysExist(initdict, "mesh_path")
        self.mesh_path = initdict["mesh_path"]

class ObjectContainer(ListManager):
    def __init__(self, name, initdict):
        LoadingHelpers.checkKeysExist(initdict, "objects")
        super(ObjectContainer, self).__init__(name, None, None)  # Manually initialise list (Can be several classes)

        for k in initdict["objects"]:
            o = initdict["objects"][k]
            LoadingHelpers.checkKeysExist(o, "type")
            LoadingHelpers.checkValueValid(o["type"], "container", "object")
            if o["type"] == "container":
                self.Elements.append(ObjectContainer(k, o))
            elif o["type"] == "object":
                self.Elements.append(Object(k, o))



'''
MAP CLASSES
'''

class Entity():
    def __init__(self, name, initdict):
        self.Name = name
        self.Position = Position(initdict["position"])
        self.Shape = Shape(initdict["shape"])

        self.Chest = initdict["chest"] if "chest" in initdict else False # TODO
        self.Trajectory = Trajectory(initdict["trajectory"])
        self.CurrentPath = []

    def setCurrentPath(self, path):
        self.CurrentPath = path


class Zone():
    def __init__(self, name, initdict):
        self.Name = name
        self.Position = Position(initdict["position"])
        self.Shape = Shape(initdict["shape"])
        self.Visual = Visual(initdict["visual"])

        self.Walkable = initdict["properties"]["walkable"]


class Waypoint():
    def __init__(self, name, initdict):
        self.Name = name
        self.Position = Position(initdict["position"])


class Object():
    def __init__(self, name, initdict):
        self.Name = name
        self.Position = Position(initdict["position"])
        self.Shape = Shape(initdict["shape"])
        self.Visual = Visual(initdict["visual"])
        self.Type = initdict["type"]

        self.Chest = initdict["chest"] if "chest" in initdict else False # TODO
        self.UserData = initdict["userdata"]
