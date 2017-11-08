#!/usr/bin/python
import rospy

class MapElement(object):
    def __init__(self, classname):
        self.ClassName = classname

    def get(self, mappath):  # To be overritten.
        raise NotImplementedError("The get method needs to be overwritten.")

    def set(self, mappath, new_value_dict):  # To be overritten.
        raise NotImplementedError("The set method needs to be overwritten.")

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
    def __init__(self, listname, classdef, initdict):
        super(ListManager, self).__init__(listname)
        self.Elements = []
        if initdict:  # Let the user create an empty list (override the loading process)
            for k in initdict.keys():
                self.Elements.append(classdef(k, initdict[k]))

    def get(self, mappath):
        key = mappath.getNextKey()
        if len(self.Elements):
            if key.Extension == self.Elements[0].ClassName: #TODO Hardcoded :/
                for e in self.Elements:
                    if key.KeyName == e.Name:
                        return e.get(mappath)
                rospy.logerr("[memory/map] GET request: ObjectContainer didn't find any object named '{}'".format(key.KeyName))
                return None
            else:
                rospy.logerr("[memory/map] GET request: ObjectContainer couldn't recognize key extension '{}'".format(key.Extension))
        else:
            return None
