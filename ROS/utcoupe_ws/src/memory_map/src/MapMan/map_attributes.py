#!/usr/bin/python
from map_loader import LoadingHelpers
from map_bases import ListManager, DictManager, NakedDictManager


class Position2D(NakedDictManager):
    def __init__(self, initdict):
        LoadingHelpers.checkKeysExist(initdict, "frame_id", "x", "y", "type")
        super(Position2D, self).__init__(initdict)


class Shape2D(NakedDictManager):
    def __init__(self, initdict):
        LoadingHelpers.checkKeysExist(initdict, "type")
        # TODO : validate for each shape type
        super(Shape2D, self).__init__(initdict)


class MarkerRViz(NakedDictManager):
    def __init__(self, initdict):
        LoadingHelpers.checkKeysExist(initdict, "ns", "id", "type", "scale", "z", "orientation", "color")
        # TODO let some arguments be optional (orientation not given -> 0, 0, 0)
        super(MarkerRViz, self).__init__(initdict)


class Trajectory(): # TODO Inherit
    def __init__(self, initdict):
        pass
