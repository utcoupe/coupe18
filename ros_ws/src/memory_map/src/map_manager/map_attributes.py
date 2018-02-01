#!/usr/bin/python
import math
import map
from map_loader import LoadingHelpers
from map_bases import DictManager


class Position2D(DictManager):
    def __init__(self, initdict):
        LoadingHelpers.checkKeysExist(initdict, "frame_id", "x", "y", "type")
        super(Position2D, self).__init__(initdict)

    def transform(self, codes):
        if "x_mirror" in codes:
            self.Dict["x"] = map.Map.get("/terrain/shape/^").Dict["width"] - self.Dict["x"]
            if "a" in self.Dict.keys():
                self.Dict["a"] = math.pi - self.Dict["a"]
        return True

class Shape2D(DictManager):
    def __init__(self, initdict):
        LoadingHelpers.checkKeysExist(initdict, "type")
        # TODO : validate for each shape type
        super(Shape2D, self).__init__(initdict)


class MarkerRViz(DictManager):
    def __init__(self, initdict):
        LoadingHelpers.checkKeysExist(initdict, "ns", "id", "type", "scale", "z", "orientation", "color")
        # TODO let some arguments be optional (orientation not given -> 0, 0, 0)
        super(MarkerRViz, self).__init__(initdict)


class Trajectory(): # TODO Inherit
    def __init__(self, initdict):
        pass
