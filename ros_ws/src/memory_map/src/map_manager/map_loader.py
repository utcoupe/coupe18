#!/usr/bin/python
import os
import xml.etree.ElementTree as ET
import rospy

from memory_definitions.srv import GetDefinition

class LoadChecker():
    @staticmethod
    def checkNodesExist(xml, *node_names):
        for node in node_names:
            if not len(xml.findall(node)) > 0:
                raise KeyError("ERROR While loading map : '{}' node name required inside '{}' node.".format(node, xml.tag))

    @staticmethod
    def checkAttribsExist(xml, *attrib_names):
        for attrib in attrib_names:
            if not attrib in xml.attrib:
                raise KeyError("ERROR While loading map : '{}' attribute required inside '{}' node.".format(attrib, xml.tag))

class MapLoader():
    CONFIG_FILE  = "config.xml"
    CLASSES_FILE = "classes.xml"
    OBJECTS_FILE = "objects.xml"
    ROBOTS_FILE  = "robots.xml"

    @staticmethod
    def loadFile(filename):
        return MapLoader.loafXMLFromDescriptionModule(filename)

    @staticmethod
    def loafXMLFromDescriptionModule(filename):
        '''
        Loads the description file by getting it from the 'memory/definitions' ROS package.
        '''
        get_def = rospy.ServiceProxy('/memory/definitions/get', GetDefinition)
        try:
            get_def.wait_for_service(timeout = 2)
        except:
            rospy.logerr("FATAL Could not contact definitions service, timeout reached. Aborting.")
            raise Exception()

        try:
            res = get_def('map/' + filename)
            if not res.success:
                rospy.logerr("Error when fetching '{}' definition file".format(filename))
                raise Exception()
            return MapLoader._xml_from_file(res.path)
        except rospy.ServiceException as exc:
            rospy.logerr("Unhandled error while getting def file: {}".format(str(exc)))
            raise Exception()

    @staticmethod
    def _xml_from_file(path):
        try:
            return ET.parse(path).getroot()
        except Exception as exc:
            rospy.logerr("Could not load map XML file : " + str(exc))
