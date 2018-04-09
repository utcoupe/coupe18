#!/usr/bin/python2.7
# -*-coding:Utf-8 -*

import rospy
import actuators

__author__ = "P. Potiron", "Thomas Fuhrmann"
__date__ = 9/94/2018

NODE_NAME = "actuators"


class ActuatorsNode():
    def __init__(self):
        rospy.init_node(NODE_NAME, log_level=rospy.DEBUG)
        self.dispatch_instance = actuators.ActuatorsDispatch()
        rospy.spin()


if __name__ == '__main__':
    ActuatorsNode()
