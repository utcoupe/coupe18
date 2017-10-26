#!/usr/bin/python2.7
# -*-coding:Utf-8 -*

import xml.etree.ElementTree as ET
import os

xml_path = os.path.dirname(__file__) + '/../definitions/actuators.xml'
xml_root = ET.parse(xml_path).getroot()

#ActuatorsProperties
class ActuatorsProperties():
    """Explore Actuators' Properties"""
    def __init__(self,name=None,id=None):
        for child in xml_root.iter('act'):
            if child.get('id')==id or child.get('name')==name:
                self.xml_actuators=child
                break
        self.id = self.xml_actuators.get('id')
        self.name = self.xml_actuators.get('name')
        self.type = self.xml_actuators.get('type')
        self.family = self.xml_actuators.get('family')
        self.default_timeout=self.xml_actuators.find('timeout').get('value')
        positions = self.xml_actuators.find('positions')
        self.default_position = positions.get('default')
        self.preset = {}
        for preset in positions.findall('preset'):
            self.preset[preset.get('name')] = preset.get('value')
    
# Test of the class
if __name__ == '__main__':
    xml = ActuatorsProperties(name='gripper_1');
    print 'id: ' + xml.id
    print 'name: ' + xml.name
    print 'type: ' + xml.type
    print 'family: ' + xml.family
    print 'default_timeout: ' + xml.default_timeout
    print 'default_position: ' + xml.default_position
    print 'presets: '
    for name, value in xml.preset.items():
        print '\t' + name + ': ' + value