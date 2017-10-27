#!/usr/bin/python
import rospy
from loader import LoadingHelpers
import map_objects

class MapElement(object):
    def __init__(self, name):
        self.Name = name

'''
class ListGroup(MapElement):
    def __init__(self, name, initdict, listsdict):
        super(ListGroup, self).__init__(name)
        LoadingHelpers.checkKeysExist(initdict, *listsdict.keys())

        self.Lists = []
        for k in listsdict.keys():
            self.Lists.append(ListManager(k, listsdict[k], initdict[k]))
'''

class ListManager(MapElement):
    def __init__(self, name, classtype, initdict):
        super(ListManager, self).__init__(name)
        self.Elements = []
        for k in initdict.keys():
            self.Elements.append(classtype(k, initdict[k]))


class Terrain():
    def __init__(self, name, initdict):
        '''
        description = {
            "zones": map_objects.Zone,
            "waypoints": map_objects.Waypoint
        }
        super(Terrain, self).__init__(name, initdict, description)
        '''
        LoadingHelpers.checkKeysExist(initdict, "walls", "zones", "waypoints")
        #self.Walls = ListManager("zones", map_objects.Zone, initdict["zones"])  # TODO: implement
        self.Zones = ListManager("zones", map_objects.Zone, initdict["zones"])
        self.Waypoints = ListManager("waypoints", map_objects.Waypoint, initdict["waypoints"])
