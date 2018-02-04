#!/usr/bin/python
from map_loader import LoadingHelpers
from map_bases import DictManager
from map_attributes import Position2D, Shape2D, MarkerRViz, Trajectory
import map


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
            "marker": MarkerRViz(initdict["marker"], shape = Shape2D(initdict["shape"])),
            "properties": DictManager(initdict["properties"])
        })


class Waypoint(DictManager):
    def __init__(self, initdict):
        LoadingHelpers.checkKeysExist(initdict, "position")
        super(Waypoint, self).__init__({
            "position": Position2D(initdict["position"])
        })


class Entity(DictManager):
    def __init__(self, initdict, obj_classes):
        LoadingHelpers.checkKeysExist(initdict, "position", "shape", "marker", "containers", "trajectory")

        for container in initdict["containers"]:
            initdict["containers"][container] = Container(initdict["containers"][container], obj_classes)

        super(Entity, self).__init__({
            "position": Position2D(initdict["position"]),
            "shape": Shape2D(initdict["shape"]),
            "marker": MarkerRViz(initdict["marker"]),
            "chest": DictManager(initdict["containers"]),
            "trajectory": Trajectory(initdict["trajectory"])
        })


class Container(DictManager):
    def __init__(self, initdict, obj_classes):
        for obj in initdict:
            initdict[obj] = Object(initdict[obj], obj_classes)
        super(Container, self).__init__(initdict)


class Object(DictManager):
    def __init__(self, initdict, obj_classes):
        LoadingHelpers.checkKeysExist(initdict, "position", "shape", "marker")

        # Autofilling if class is available
        if "class" in initdict:
            obj_class = [obj_classes[d] for d in obj_classes if d == initdict["class"]][0]

        d = {}
        if "color" in initdict:
            d["color"] = [c for c in map.Map.Colors if c.Dict["name"] == initdict["color"]][0]
        if "properties" in initdict:
            d["properties"] = DictManager(initdict["properties"])
        d["shape"] = Shape2D(initdict["shape"])

        d = {
            "position": Position2D(initdict["position"]),
            "marker": MarkerRViz(initdict["marker"], shape = d["shape"] if "shape" in d else None, \
                                                     color = d["color"] if "color" in d else None)
        }
        super(Object, self).__init__(d)
