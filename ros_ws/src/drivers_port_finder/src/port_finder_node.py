#!/usr/bin/env python

import os
import rospy
import xml.etree.ElementTree as ET
from drivers_port_finder.srv import *

__author__ = "Thomas Fuhrmann"
__date__ = 18/01/2017

NODE_NAME = "port_finder"


class PortFinder:
    def __init__(self):
        rospy.logdebug("[drivers/port_finder] Starting the node.")
        self._components_list = ()
        # Init ROS stuff
        rospy.init_node(NODE_NAME, anonymous=False, log_level=rospy.INFO)
        self._srv_goto = rospy.Service("/drivers/" + NODE_NAME + "/get_port", GetPort, self._callback_get_port)
        curr_dir = os.path.dirname(os.path.abspath(__file__))
        def_dir = os.path.join(curr_dir, "../def")
        self._parse_xml_file(def_dir + "/components_list.xml")

    def _callback_get_port(self, request):
        return GetPortResponse(response)

    def _parse_xml_file(self, file):
        rospy.logdebug("Parsing port_finder definition...")
        try:
            root = ET.parse(file).getroot()
        except:
            rospy.logerr("File {} not found...".format(file))
            root = None
        components = []
        if root is not None:
            for component in root.findall("component"):
                if "name" not in component.attrib:
                    rospy.logerr("Can't parse component: a 'name' attribute is required. Skipping this component.")
                    continue

                required = ["name", "vendor_id", "product_id", "port_type"]
                for param in required:
                    if component.attrib[param] is None:
                        rospy.logerr("Can't parse component definition: a '{}' element is required. Skipping this component.".format(p))

                components.append({
                    "name": component.attrib["name"],
                    "vendor_id": component.attrib["vendor_id"],
                    "product_id": component.attrib["product_id"],
                    "port_type": component.attrib["port_type"]
                })

            if not components:
                rospy.logwarn("No component found in component_list definition.")

            rospy.loginfo("{} components found in component_list definition".format(len(components)))
            rospy.loginfo(components)
            self._components_list = components


if __name__ == "__main__":
    PortFinder()
