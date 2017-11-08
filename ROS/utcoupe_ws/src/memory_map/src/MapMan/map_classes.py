#!/usr/bin/python
from map_bases import DictManager, ListManager
from map_attributes import Position, Shape2D, Visual, Color


class Terrain(DictManager):
    def __init__(self, initdict):
        super(Terrain, self).__init__({
            "shape": Shape2D(initdict["shape"]),
            "visual": Visual(initdict["shape"]),
            "walls": ListManager(Shape2D, initdict["shape"]),
        })


class Zones(DictManager):
    def __init__(self, initdict):
        super(Zones, self).__init__({
            "shape": Shape2D(initdict["shape"]),
            "visual": Visual(initdict["shape"]),
            "walls": ListManager(Shape2D, initdict["shape"]),
        })


class Waypoints(DictManager):
    def __init__(self, initdict):
        super(Waypoints, self).__init__({
            "shape": Shape2D(initdict["shape"]),
            "visual": Visual(initdict["shape"]),
            "walls": ListManager(Shape2D, initdict["shape"]),
        })


class Entities(DictManager):
    def __init__(self, initdict):
        super(Entities, self).__init__({
            "shape": Shape2D(initdict["shape"]),
            "visual": Visual(initdict["shape"]),
            "walls": ListManager(Shape2D, initdict["shape"]),
        })


class Objects(DictManager):
    def __init__(self, initdict):
        super(Objects, self).__init__({
            "shape": Shape2D(initdict["shape"]),
            "visual": Visual(initdict["shape"]),
            "walls": ListManager(Shape2D, initdict["shape"]),
        })
