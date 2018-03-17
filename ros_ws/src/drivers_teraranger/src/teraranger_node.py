#!/usr/bin/env python

import subprocess
import rospy
from drivers_ard_others.msg import *
from drivers_port_finder.srv import *
from sensor_msgs.msg import Range
from numpy import inf

__author__ = "Thomas Fuhrmann"
__date__ = 06/01/2018

NODE_NAME = "teraranger"
PUBLISH_INTERVAL = 0.1  # in ms
GET_PORT_SERVICE_NAME = "/drivers/port_finder/get_port"
GET_PORT_SERVICE_TIMEOUT = 15  # in seconds


class Teraranger:
    def __init__(self):
        rospy.init_node(NODE_NAME, anonymous=False, log_level=rospy.INFO)
        teraranger_port = ""
        node_subprocess = None
        try:
            rospy.wait_for_service(GET_PORT_SERVICE_NAME, GET_PORT_SERVICE_TIMEOUT)
            self._src_client_get_port = rospy.ServiceProxy(GET_PORT_SERVICE_NAME, GetPort)
            teraranger_port = self._src_client_get_port("teraranger").port
        except rospy.ROSException:
            rospy.logerr("Port_finder service does not exist, can't fetch the teraranger port...")
        if teraranger_port != "":
            rospy.logwarn("Teraranger port : " + teraranger_port)
            node_subprocess = subprocess.Popen(["rosrun", "teraranger", "one", "_portname:=" + teraranger_port, "__ns:=/"])
        else:
            rospy.logerr("Teraranger port has not been found, start the node but can't send real data...")
        self._range_value = 0.0
        self._pub_belt_range = rospy.Publisher("/drivers/ard_others/belt_ranges", BeltRange, queue_size=1)
        self._sub_terarange = rospy.Subscriber("/teraranger_one", Range, self._callback_teranranger_range)
        while not rospy.is_shutdown():
            self._pub_belt_range.publish(BeltRange("sensor_tera1", self._range_value))
            rospy.sleep(PUBLISH_INTERVAL)
        if node_subprocess:
            node_subprocess.terminate()

    def _callback_teranranger_range(self, data):
        if data.range == -inf:
            data.range = -1
        self._range_value = float(data.range)


if __name__ == "__main__":
    Teraranger()
