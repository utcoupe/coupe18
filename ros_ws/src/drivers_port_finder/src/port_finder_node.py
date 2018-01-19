#!/usr/bin/env python

import os
import re
import subprocess
import serial
import rospy
import xml.etree.ElementTree as ET
from drivers_port_finder.srv import *

__author__ = "Thomas Fuhrmann"
__date__ = 18/01/2017

NODE_NAME = "port_finder"
ARDUINO_LIST = ("mega", "nano", "uno", "leo")


class PortFinder:
    def __init__(self):
        rospy.logdebug("[drivers/port_finder] Starting the node.")
        # List containing the content of the xml definition file
        self._components_list = []
        # List containing the connected components and the corresponding port
        # TODO use a dictionary instead ?
        self._connected_component_list = []
        # List containing the final processing information, matching the serial port and the serial component
        self._associated_port_list = []
        # Init ROS stuff
        rospy.init_node(NODE_NAME, anonymous=False, log_level=rospy.INFO)
        self._srv_goto = rospy.Service("/drivers/" + NODE_NAME + "/get_port", GetPort, self._callback_get_port)
        curr_dir = os.path.dirname(os.path.abspath(__file__))
        def_dir = os.path.join(curr_dir, "../def")
        self._parse_xml_file(def_dir + "/components_list.xml")
        self._parse_dmesg()
        self._associate_port()
        self._identify_arduino()
        rospy.loginfo(self._associated_port_list)

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
            rospy.logdebug("{} components found in component_list definition".format(len(components)))
            # rospy.loginfo(components)
            self._components_list = components

    def _parse_dmesg(self):
        dmesg_output = self._get_dmesg().split('\n')
        id_list = []
        tty_list = []
        id_list_filtered = []
        merged_filtered_id_tty_list = []
        line_number = 0
        tty_regexp = re.compile('(ttyACM.|ttyUSB.)')
        # Parse the whole file to extract a list of lines containing idVendor and an other list containing ttyX
        for line in dmesg_output:
            vendor_id_index = line.find("idVendor")
            tty_usb_index = line.find("ttyUSB")
            tty_acm_index = line.find("ttyACM")
            if vendor_id_index != -1:
                matched_line = line[vendor_id_index:len(line)]
                matched_line_split = matched_line.split(', ')
                # line_number, vendor_id, component_id
                id_list.append((line_number, matched_line_split[0].split('=')[1], matched_line_split[1].split('=')[1]))
            if tty_usb_index != -1 or tty_acm_index != -1:
                matched_tty = tty_regexp.search(line)
                # line_number, ttyX
                tty_list.append((line_number, matched_tty.group(0)))
            line_number += 1
        # Filter the idVendor list to keep only the idVendor we use
        for element in id_list:
            for component in self._components_list:
                if element[1] == component["vendor_id"] and element[2] == component["product_id"]:
                    id_list_filtered.append(element + (component["name"],))
        # TODO filter the list to keep only the most recent vendor_id
        # Merge the information of vendor_id and tty port to have a single tuple in list
        for element in id_list_filtered:
            for tty_element in tty_list:
                if tty_element[0] > element[0]:
                    merged_filtered_id_tty_list.append((element[1], element[2], tty_element[1], element[3]))
                    break
        rospy.loginfo("ID_LIST")
        rospy.loginfo(id_list_filtered)
        rospy.loginfo("TTY_LIST")
        rospy.loginfo(tty_list)
        rospy.loginfo("MERGED_LIST")
        rospy.loginfo(merged_filtered_id_tty_list)
        self._connected_component_list = merged_filtered_id_tty_list

    def _get_dmesg(self):
        """
        From https://gist.github.com/saghul/542780
        """
        try:
            sub = subprocess.Popen("dmesg", stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            stdout, stderr = sub.communicate()
            returncode = sub.returncode
        except OSError, e:
            if e.errno == 2:
                raise RuntimeError('"dmesg" is not present on this system')
            else:
                raise
        if returncode != 0:
            raise RuntimeError('Got return value %d while executing "%s", stderr output was:\n%s' % (
            returncode, " ".join("dmesg"), stderr.rstrip("\n")))
        return stdout

    def _associate_port(self):
        for element in self._connected_component_list:
            self._associated_port_list.append((element[3], "/dev/" + element[2]))
        rospy.loginfo(self._associated_port_list)

    # translate the arduino type in the real arduino connected
    def _identify_arduino(self):
        for counter, element in enumerate(self._associated_port_list):
            if element[0].find("ard") != -1:
                rospy.loginfo("Found an arduino to open !")
                arduino_id = ""
                try:
                    com_line = serial.Serial(element[1], 57600, timeout=1.2)
                    # First line may be empty...
                    com_line.readline()
                    arduino_id = com_line.readline()
                    com_line.close()
                except:
                    rospy.logwarn("Try to open port {} but it fails...".format(element[1]))
                if arduino_id.find("\r\n") > -1:
                    arduino_id = arduino_id.replace("\r\n", "")
                    self._associated_port_list[counter] = (arduino_id, element[1])
                    rospy.loginfo("Found arduino : " + arduino_id)


if __name__ == "__main__":
    PortFinder()
