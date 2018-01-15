#!/usr/bin/python2.7
# -*-coding:Utf-8 -*

import xml.etree.ElementTree as ET
import os

_xml_path = os.path.dirname(__file__) + '/../def/actuators.xml'
_actuators_list = None
#ActuatorsProperties
class ActuatorsProperties():
    """Explore Actuators' Properties"""

    def __init__(self, xml_actuators):
        self.id = xml_actuators.get('id')
        self.name = xml_actuators.get('name')
        self.type = xml_actuators.get('type')
        self.family = xml_actuators.get('family')
        self.default_timeout=xml_actuators.find('timeout').get('value')
        positions = xml_actuators.find('positions')
        self.default_position = positions.get('default')
        self.preset = {}
        for preset in positions.findall('preset'):
            self.preset[preset.get('name')] = preset.get('value')


def initActuatorsList():
    global _actuators_list, _xml_path
    xml_root = ET.parse(_xml_path).getroot()
    _actuators_list = {}
    for child in xml_root.iter('act'):
        _actuators_list[child.get('name')] = ActuatorsProperties(child)

def getActuatorsList():
    global _actuators_list
    return _actuators_list

if __name__ == '__main__':
    initActuatorsList()
    print getActuatorsList()
