#!/usr/bin/python

class MapElement(object):
    def __init__(self, name):
        self.Name = name

    def get(self, mappath):  # To be overritten.
        pass

    def set(self, mappath, new_value):  # To be overritten.
        pass

'''
CONSTRUCTOR
description = {
    "zones": map_objects.Zone,
    "waypoints": map_objects.Waypoint
}
super(Terrain, self).__init__(name, initdict, description)

DEF
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
        if initdict:  # Let the user create an empty list (override the loading process)
            for k in initdict.keys():
                self.Elements.append(classtype(k, initdict[k]))
