#!/usr/bin/python
from map_loader import LoadingHelpers
from map_bases import DictManager, ListManager
from map_attributes import Position2D, Shape2D, MarkerRViz, Trajectory


class Terrain(DictManager):
    def __init__(self, initdict):
        LoadingHelpers.checkKeysExist(initdict, "shape", "marker", "walls")
        super(Terrain, self).__init__({
            "shape": Shape2D(initdict["shape"]),
            "marker": MarkerRViz(initdict["marker"]),
            "walls": ListManager(Shape2D, initdict["walls"]),
        })


class Zone(DictManager):
    def __init__(self, initdict):
        LoadingHelpers.checkKeysExist(initdict, "position", "shape", "marker", "properties")
        super(Zone, self).__init__({
            "position": Position2D(initdict["position"]),
            "shape": Shape2D(initdict["shape"]),
            "marker": MarkerRViz(initdict["marker"]),
            "properties": DictManager(initdict["properties"])
        })


class Waypoint(DictManager):
    def __init__(self, initdict):
        LoadingHelpers.checkKeysExist(initdict, "position", "marker")
        super(Waypoint, self).__init__({
            "position": Position2D(initdict["position"]),
            "marker": MarkerRViz(initdict["marker"])
        })


class Entity(DictManager):
    def __init__(self, initdict):
        LoadingHelpers.checkKeysExist(initdict, "position", "shape", "marker", "chest", "trajectory")
        super(Entity, self).__init__({
            "position": Position2D(initdict["position"]),
            "shape": Shape2D(initdict["shape"]),
            "marker": MarkerRViz(initdict["marker"]),
            "chest": None, # TODO Implement
            "trajectory": Trajectory(initdict["trajectory"])
        })


class Object(DictManager):
    def __init__(self, initdict):
        LoadingHelpers.checkKeysExist(initdict, "type", "position", "shape", "marker", "properties")
        super(Object, self).__init__({
            "type": initdict["type"],
            "position": Position2D(initdict["position"]),
            "shape": Shape2D(initdict["shape"]),
            "marker": MarkerRViz(initdict["marker"]),
            "properties": DictManager(initdict["properties"]),
        })
