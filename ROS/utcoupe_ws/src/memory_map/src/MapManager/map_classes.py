#!/usr/bin/python
from loader import LoadingHelpers
from map_bases import MapElement, ListManager
from map_objects import *


class Terrain(MapElement):
    '''
    Terrain Metaclass. Holds the map general information, as well as
    general objects : Zones and Waypoints.
    '''
    def __init__(self, initdict):
        super(Terrain, self).__init__("terrain")
        LoadingHelpers.checkKeysExist(initdict, "shape", "visual", "walls")
        self.Shape = Shape(initdict["shape"])
        self.Visual = Visual(initdict["visual"])
        self.Walls = initdict["walls"]  # self.Walls = ListManager("zones", map_objects.Zone, initdict["zones"])  # TODO implement


class Entity(MapElement):
    '''
    Describes a dynamic robot, buddy or enemy. Holds information like
    the current robot's path and state, previous trajectories, containers, etc.

    Can check conditions on the robot's position, etc.
    '''

    def __init__(self, name, initdict):
        super(Entity, self).__init__("entity")
        self.Name = name # TODO mixing entity name and class type xwith self.Name in MapElement
        LoadingHelpers.checkKeysExist(initdict, "position", "shape", "visual", "trajectory")
        self.Position = Position(initdict["position"])
        self.Shape = Shape(initdict["shape"])
        self.Visual = Visual(initdict["visual"])

        self.Chest = initdict["chest"] if "chest" in initdict else False # TODO
        self.Trajectory = Trajectory(initdict["trajectory"])
        self.CurrentPath = []

    def get(self, mappath):
        print "got until here"
        key = mappath.getNextKey()
        if key.Extension == "attribute":
            if key.KeyName == "position":
                return self.Position.get()
            elif key.KeyName == "shape":
                return self.Shape.get()
            elif key.KeyName == "visual":
                return self.Visual.get()
            elif key.KeyName == "chest": #TODO complex ?
                return self.Chest
            else:
                rospy.logerr("[memory/map] GET request: Entity didn't find any attribute named '{}'".format(key.KeyName))
                return None
        else:
            rospy.logerr("[memory/map] GET request: Entity couldn't recognize key extension '{}'".format(key.Extension))


class Zone(MapElement):
    '''
    Describes a zone : can be useful for checking conditions (e.g. if an object or entity
    is in a certain area).

    Can hold additional information, like speed limits or walkability.
    '''

    def __init__(self, name, initdict):
        LoadingHelpers.checkKeysExist(initdict, "position", "shape", "visual", "properties")
        super(Zone, self).__init__("zone")
        self.Name = name
        self.Position = Position(initdict["position"])
        self.Shape = Shape(initdict["shape"])
        self.Visual = Visual(initdict["visual"])
        self.Properties = initdict["properties"]

    def get(self, mappath):
        key = mappath.getNextKey()
        if key.Extension == "attribute":
            if key.KeyName == "position":
                return self.Position.get()
            elif key.KeyName == "shape":
                return self.Shape.get()
            elif key.KeyName == "visual":
                return self.Visual.get()
            elif key.KeyName == "properties":
                return self.Properties
            else:
                rospy.logerr("[memory/map] GET request: Object didn't find any attribute named '{}'".format(key.KeyName))
                return None
        else:
            rospy.logerr("[memory/map] GET request: Object couldn't recognize key extension '{}'".format(key.Extension))


class Waypoint(MapElement):
    '''
    Describes a static position that was given a name. Can be gotten from other packages
    for referencing a XY position while defining it only once here.

    Can hold additional information, like an approach circle.
    '''

    def __init__(self, name, initdict):
        super(Waypoint, self).__init__("waypoint")
        self.Name = name
        self.Position = Position(initdict["position"])
    
    def get(self, mappath):
        key = mappath.getNextKey()
        if key.Extension == "attribute":
            if key.KeyName == "position":
                return self.Position.get()
            else:
                rospy.logerr("[memory/map] GET request: Object didn't find any attribute named '{}'".format(key.KeyName))
                return None
        else:
            rospy.logerr("[memory/map] GET request: Object couldn't recognize key extension '{}'".format(key.Extension))


class ObjectContainer(ListManager):
    '''
    Manages a list of objects. Can handle transfers of object from a
    container to another, and check conditions.
    '''

    def __init__(self, name, initdict):
        LoadingHelpers.checkKeysExist(initdict, "objects")
        super(ObjectContainer, self).__init__("objects", None, None)  # Manually initialise list (Can be several classes)
        self.Name = name

        for k in initdict["objects"]:
            o = initdict["objects"][k]
            LoadingHelpers.checkKeysExist(o, "type")
            LoadingHelpers.checkValueValid(o["type"], "container", "object")
            if o["type"] == "container":
                self.Elements.append(ObjectContainer(k, o))
            elif o["type"] == "object":
                self.Elements.append(Object(k, o))

    def get(self, mappath):
        key = mappath.getNextKey()
        if key.Extension == "object":
            for o in self.Elements:
                if key.KeyName == o.Name:
                    return o.get(mappath)
            rospy.logerr("[memory/map] GET request: ObjectContainer didn't find any object named '{}'".format(key.KeyName))
            return None
        else:
            rospy.logerr("[memory/map] GET request: ObjectContainer couldn't recognize key extension '{}'".format(key.Extension))


class Object(MapElement):
    def __init__(self, name, initdict):
        super(Object, self).__init__("object")
        self.Name = name
        self.Position = Position(initdict["position"])
        self.Shape = Shape(initdict["shape"])
        self.Visual = Visual(initdict["visual"])
        self.Type = initdict["type"]

        self.Chest = initdict["chest"] if "chest" in initdict else False # TODO
        self.UserData = initdict["userdata"]

    def get(self, mappath):
        key = mappath.getNextKey()
        if key.Extension == "attribute":
            if key.KeyName == "position":
                return self.Position.get()
            elif key.KeyName == "shape":
                return self.Shape.get()
            elif key.KeyName == "visual":
                return self.Visual.get()
            elif key.KeyName == "chest": #TODO complex ?
                return self.Chest
            elif key.KeyName == "userdata":
                if len(mappath.getKeysLeft()) > 0:
                    d = self.UserData
                    for k in mappath.getKeysLeft():
                        if k.KeyName in d: d = d[k.KeyName]
                        else: return None
                    return d
                else: return self.UserData
            else:
                rospy.logerr("[memory/map] GET request: Object didn't find any attribute named '{}'".format(key.KeyName))
                return None
        else:
            rospy.logerr("[memory/map] GET request: Object couldn't recognize key extension '{}'".format(key.Extension))
