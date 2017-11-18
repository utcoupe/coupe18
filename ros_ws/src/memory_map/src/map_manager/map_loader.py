#!/usr/bin/python
import os
import yaml
import rospy

class LoadingHelpers():
    '''
    Helpers static class. Provides validation methods for easier and
    cleaner YML verification handling inside the classes.
    '''

    @staticmethod
    def checkKeysExist(checkdict, *keys_required):
        '''
        Checks if the dict given has all of the keys given in keys_required.
        Pops a ROS error and stops the node if the condition is not verified.
        '''
        for k in keys_required:
            if k not in checkdict.keys():
                m = "Missing required '{}' element in Map YML description file. Couldn't load Map.".format(k)
                rospy.logerr(m)
                raise ValueError(m)

    @staticmethod
    def checkValueValid(value, *values_required):
        '''
        Checks if the the value given corresponds to one of the possibilities given in values_required.
        Pops a ROS error and stops the node if the condition is not verified.
        '''
        if value not in values_required:
            m = "Element value '{}' not valid, must be in '{}'. Couldn't load Map.".format(value, values_required)
            rospy.logerr(m)
            raise ValueError(m)


class MapLoader():
    @staticmethod
    def loadFile(filename):
        '''
        Gets the YML Map description file from the specified method
        Please change the method correspondingly to what is currently used in your package.
        '''
        return MapLoader.loadYamlFromFile(filename)

    @staticmethod
    def loadYamlFromFile(filename):
        '''
        Loads the description file simply by getting the file in disk.
        The file MUST be in the package's directory to avoid any problems.
        '''
        with open(os.path.dirname(__file__) + "/" + filename, 'r') as stream:
            try:
                return yaml.load(stream)
            except yaml.YAMLError as exc:
                rospy.logerr("Could not load map YML file : " + str(exc))

    @staticmethod
    def loafYamlFromDescriptionModule(path):
        '''
        Loads the description file by gettign it from the 'memory/definitions' ROS package.
        '''
        raise NotImplementedError  # TODO use this when Definitions package is ready instead of loading from file.
