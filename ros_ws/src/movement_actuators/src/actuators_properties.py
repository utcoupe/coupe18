#!/usr/bin/python2.7
# -*-coding:Utf-8 -*

import xml.etree.ElementTree as ET
import os

import rospy
from memory_definitions.srv import GetDefinition

_xml_path = 'movement/actuators.xml'
_actuators_list = None
#ActuatorsProperties
class ActuatorsProperties():
    """Explore Actuators' Properties"""

    def __init__(self, xml_actuators):
        self.id = int(xml_actuators.get('id'))
        self.name = xml_actuators.get('name')
        self.type = xml_actuators.get('type')
        self.family = xml_actuators.get('family')
        self.default_timeout = int( xml_actuators.find('timeout').get('value') )
        positions = xml_actuators.find('positions')
        self.default_position = positions.get('default')
        self.preset = {}
        for preset in positions.findall('preset'):
            self.preset[preset.get('name')] = int( preset.get('value') )

    def __repr__(self):
        return '{}({})[{}-{}]'.format(self.name,self.id,self.family,self.type)

def _get_xml_path(filename):
    get_def = rospy.ServiceProxy('/memory/definitions/get', GetDefinition)
    try:
        get_def.wait_for_service(10)
        res = get_def(filename)
        if not res.success:
            rospy.logerr("Error when fetching '{}' definition file".format(filename))
        return res.path
    except rospy.ServiceException as exc:
        rospy.logerr("Unhandled error while getting def file: {}".format(str(exc)))
        raise Exception()

def initActuatorsList():
    global _actuators_list, _xml_path
    xml_root = ET.parse(_get_xml_path(_xml_path)).getroot()
    _actuators_list = {}
    for child in xml_root.iter('act'):
        _actuators_list[child.get('name')] = ActuatorsProperties(child)

def getActuatorsList():
    global _actuators_list
    return _actuators_list

if __name__ == '__main__':
    initActuatorsList()
    print getActuatorsList()
