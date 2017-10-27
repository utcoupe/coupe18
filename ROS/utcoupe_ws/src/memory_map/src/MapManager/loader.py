#!/usr/bin/python
import os
import rospy

class LoadingHelpers():
    @staticmethod
    def checkKeysExist(checkdict, *keys_required):
        for k in keys_required:
            if k not in checkdict.keys():
                m = "Missing required '{}' element in Map YML description file. Couldn't load Map.".format(k)
                rospy.logerr(m)
                raise ValueError(m)
    
    @staticmethod
    def checkValueValid(value, *values_required):
        if value not in values_required:
            m = "Element value '{}' not valid, must be in '{}'. Couldn't load Map.".format(value, values_required)
            rospy.logerr(m)
            raise ValueError(m)

class MapLoader():
    @staticmethod
    def loadFile(filename):
        return MapLoader.loadYamlFromFile(filename)
        # return self.init_dict(self.loadYamlFromFile(filename))

    @staticmethod
    def loadYamlFromFile(filename):
        import yaml
        with open(os.path.dirname(__file__) + "/" + filename, 'r') as stream:
            try:
                return yaml.load(stream)
            except yaml.YAMLError as exc:
                print exc

    @staticmethod
    def loafYamlFromDescriptionModule(path):
        # TODO use this when Definitions package is ready instead of loading from file.
        pass
    '''
    def init_dict(self, data):
        # -------- Terrain
        zones = data["terrain"]["zones"]
        for zone in zones:
            zones[zone] = Zone(zone, zones[zone])

        # -------- Waypoints
        waypoints = data["terrain"]["waypoints"]
        for waypoint in waypoints:
            waypoints[waypoint] = Waypoint(waypoint, waypoints[waypoint])

        # -------- Entities
        entities = data["entities"]
        for entity in entities:
            entities[entity] = Entity(entity, entities[entity])

        # -------- Objects
        objects = data["objects"]
        for elem in objects:
            #if objects[elem]["type"] == "namespace":
            #    objects[elem] = ObjectNamespace(elem, objects[elem])
            #elif objects[elem]["type"] == "container":
            #    rospy.logerr("Object '{}' found in the map description file that doesn't belong to any namespace. object not loaded".format(elem))
            if objects[elem]["type"] == "object":
                objects[elem] = Object(elem, objects[elem])
                #rospy.logerr("Object '{}' found in the map description file that doesn't belong to any namespace. object not loaded".format(elem))

        return data
    '''