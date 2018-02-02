#!/usr/bin/python
from map_loader import LoadingHelpers
from map_bases import DictManager
from map_attributes import Position2D, Shape2D, MarkerRViz, Trajectory


class Terrain(DictManager):
    def __init__(self, initdict):
        LoadingHelpers.checkKeysExist(initdict, "shape", "marker", "walls")

        # Instantiate the layers before creating the dict
        for layer in initdict["walls"]:
            initdict["walls"][layer] = Layer(initdict["walls"][layer])

        super(Terrain, self).__init__({
            "shape": Shape2D(initdict["shape"]),
            "marker": MarkerRViz(initdict["marker"]),
            "walls": DictManager(initdict["walls"]),
        })


class Layer(DictManager):
    def __init__(self, initdict):
        self.includes = []
        if "_include" in initdict.keys():
            self.includes = [i for i in initdict["_include"]]
            del initdict["_include"]

        # Instantiate the walls before creating the dict
        for wall in initdict:
            initdict[wall] = Wall(initdict[wall])

        super(Layer, self).__init__(initdict)


class Wall(DictManager):
    def __init__(self, initdict):
        LoadingHelpers.checkKeysExist(initdict, "position", "shape")
        super(Wall, self).__init__({
            "position": Position2D(initdict["position"]),
            "shape": Shape2D(initdict["shape"])
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
        LoadingHelpers.checkKeysExist(initdict, "position")
        super(Waypoint, self).__init__({
            "position": Position2D(initdict["position"])
        })


class Entity(DictManager):
    def __init__(self, initdict):
        LoadingHelpers.checkKeysExist(initdict, "position", "shape", "marker", "containers", "trajectory")

        for container in initdict["containers"]:
            initdict["containers"][container] = Container(initdict["containers"][container])

        super(Entity, self).__init__({
            "position": Position2D(initdict["position"]),
            "shape": Shape2D(initdict["shape"]),
            "marker": MarkerRViz(initdict["marker"]),
            "chest": DictManager(initdict["containers"]),
            "trajectory": Trajectory(initdict["trajectory"])
        })


class Container(DictManager):
    def __init__(self, initdict):
        for obj in initdict:
            initdict[obj] = Object(initdict[obj])
        super(Container, self).__init__(initdict)


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
