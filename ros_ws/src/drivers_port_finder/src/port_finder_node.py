#!/usr/bin/env python

import os
import re
import subprocess
import serial
import rospy
import xml.etree.ElementTree as ET
from drivers_port_finder.srv import *
from ai_game_status import StatusServices


__author__ = "Thomas Fuhrmann"
__date__ = 18/01/2017

NODE_NAME = "port_finder"
ARDUINO_LIST = ("mega", "nano", "uno", "leo")
#TODO put it in xml file
ARDUINO_NODE_LIST = ("ard_asserv",)
SERIAL_READ_SIZE = 50


class PortFinder:
    def __init__(self):
        rospy.logdebug("Starting the node.")
        # List containing the content of the xml definition file
        self._components_list = []
        # List containing the connected components and the corresponding port
        self._connected_component_list = []
        # List containing the final processing information, matching the serial port and the serial component
        self._associated_port_list = []
        # List of file descriptor for calls to rosserial
        self._rosserial_call_list = []
        # Init ROS stuff
        rospy.init_node(NODE_NAME, anonymous=False, log_level=rospy.INFO)
        curr_dir = os.path.dirname(os.path.abspath(__file__))
        def_dir = os.path.join(curr_dir, "../def")
        self._parse_xml_file(def_dir + "/components_list.xml")
        self._parse_dmesg()
        self._associate_port()
        self._identify_arduino()
        rospy.logdebug("Node ready, found : " + str(self._associated_port_list))
        self._srv_goto = rospy.Service("/drivers/" + NODE_NAME + "/get_port", GetPort, self._callback_get_port)

        # Tell ai/game_status the node initialized successfuly.
        StatusServices("drivers", "port_finder").ready(True)

        rospy.spin()
        for rosserial_fd in self._rosserial_call_list:
            rosserial_fd.terminate()

    def _callback_get_port(self, request):
        response = ""
        if request.component == "all":
            response = str(self._associated_port_list)
        elif request.component in dict(self._associated_port_list).keys():
            response = dict(self._associated_port_list)[request.component]
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
                required = ["name", "vendor_id", "product_id", "port_type", "rosserialable"]
                for param in required:
                    if component.attrib[param] is None:
                        rospy.logerr("Can't parse component definition: a '{}' element is required. Skipping this component.".format(p))
                components.append({
                    "name": component.attrib["name"],
                    "vendor_id": component.attrib["vendor_id"],
                    "product_id": component.attrib["product_id"],
                    "port_type": component.attrib["port_type"],
                    "rosserialable": component.attrib["rosserialable"]
                })
            if not components:
                rospy.logwarn("No component found in component_list definition.")
            rospy.logdebug("{} components found in component_list definition".format(len(components)))
            self._components_list = components

    def _parse_dmesg(self):
        dmesg_output = self._get_dmesg().split('\n')
        id_dict = {}
        tty_dict = {}
        id_dict_filtered = {}
        merged_filtered_id_tty_list = []
        vendor_id_regex = re.compile('usb (.*):.*idVendor=([a-z0-9]+).*idProduct=([a-z0-9]+)')
        tty_regexp = re.compile(' ([0-9\-.]+):.*(ttyACM.|ttyUSB.)')
        # Parse the whole file to extract a list of lines containing idVendor and an other list containing ttyX
        for line in dmesg_output:
            vendor_id_matched = vendor_id_regex.search(line)
            tty_matched = tty_regexp.search(line)
            if vendor_id_matched is not None:
                id_dict[vendor_id_matched.group(1)] = (vendor_id_matched.group(2), vendor_id_matched.group(3))
            if tty_matched is not None:
                tty_dict[tty_matched.group(1)] = tty_matched.group(2)
        # Filter the idVendor list to keep only the idVendor we use
        for element in id_dict:
            for component in self._components_list:
                if id_dict[element][0] == component["vendor_id"] and id_dict[element][1] == component["product_id"]:
                    id_dict_filtered[element] = (id_dict[element] + (component["name"],))
        # Merge the information of vendor_id and tty port to have a single tuple in list
        for element in id_dict_filtered:
            for tty_element in tty_dict:
                if tty_element == element:
                    merged_filtered_id_tty_list.append((id_dict_filtered[element][0], id_dict_filtered[element][1], tty_dict[tty_element], id_dict_filtered[element][2]))
                    break
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

    def _identify_arduino(self):
        rosserial_port_list = []
        for counter, element in enumerate(self._associated_port_list):
            if self._check_rosseriable(element[0]):
                read_data = ""
                serial_port_disconnected = False
                arduino_node_flag = False
                try:
                    com_line = serial.Serial(element[1], 57600, timeout=5)
                    read_data = com_line.read(SERIAL_READ_SIZE)
                    com_line.close()
                except serial.SerialException:
                    rospy.logerr("Try to open port {} but it fails...".format(element[1]))
                    serial_port_disconnected = True
                if not serial_port_disconnected:
                    # Check if it's an arduino using UTCoupe protocol
                    for arduino_node in ARDUINO_NODE_LIST:
                        if read_data.find(arduino_node) != -1:
                            arduino_node_flag = True
                            self._associated_port_list[counter] = (arduino_node, element[1])
                            rospy.loginfo("Found a real arduino named " + arduino_node)
                            arduino_node_flag = True
                    # Otherwise, in any case, start rosserial
                    if not arduino_node_flag:
                        rospy.loginfo("Found an arduino to start with rosserial : " + element[1] + ", start it.")
                        self._rosserial_call_list.append(subprocess.Popen(["rosrun", "rosserial_python", "serial_node.py", element[1], "__name:=serial_node_" + str(counter)]))
                        rosserial_port_list.append(element[1])
                        # Replace the tuple in list to keep a track that the port is used by rosserial
                        # Add an arbitrary id to rosserial to avoid having 2 components with the same name
                        self._associated_port_list[counter] = ("rosserial_node_" + str(counter), element[1])

    def _check_rosseriable(self, component_name):
        returned_value = False
        if component_name != "":
            for element in self._components_list:
                if element["name"] == component_name:
                    if element["rosserialable"] == "true":
                        returned_value = True
        return returned_value


if __name__ == "__main__":
    PortFinder()
