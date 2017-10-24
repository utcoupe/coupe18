#!/usr/bin/python
import rospy
from map_loader import MapLoader


class Map():
    def __init__(self, filename):
        self.MapDict = MapLoader().load(filename)

    def get(self, filter):
        pass

    def set(self, filter, value):
        pass

    def getMapDict(self):
        return self.MapDict

    def getObject(self, objectname):
        for elem in self.MapDict["objects"]:
            o = elem.getObject(objectname)
            if o: return o
        return None
