#!/usr/bin/env python

import rospy
from drivers_ard_others.msg import *
from sensor_msgs.msg import Range
from numpy import inf
__author__ = "Thomas Fuhrmann"
__date__ = 06/01/2018

NODE_NAME = "ard_others_teraranger"

#rosrun teraranger one _portname:=/dev/tUSB0

class ArdOthersTeraranger:
    def __init__(self):
        rospy.init_node(NODE_NAME, anonymous=False, log_level=rospy.INFO)
        self._range_value = 0.0
        self._pub_belt_range = rospy.Publisher("/drivers/ard_others/belt_ranges", BeltRange, queue_size=1)
        self._sub_terarange = rospy.Subscriber("/teraranger_one", Range, self._callback_teranranger_range)
        while not rospy.is_shutdown():
            self._pub_belt_range.publish(BeltRange("sensor_tera", self._range_value))
            rospy.sleep(0.1)

    def _callback_teranranger_range(self, data):
        if data.range == -inf:
            data.range = -1
        self._range_value = float(data.range)


if __name__ == "__main__":
    ArdOthersTeraranger()
