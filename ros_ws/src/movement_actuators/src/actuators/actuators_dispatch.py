#!/usr/bin/env python

import rospy
import actionlib
from actuators_abstract import *
from movement_actuators.msg import *

__author__ = "P. Potiron", "Thomas Fuhrmann"
__date__ = 9/04/2018


class ActuatorsDispatch(ActuatorsAbstract):
    def __init__(self):
        ActuatorsAbstract.__init__(self, "dispatch", DispatchAction)
        rospy.loginfo("Dispatch constructed")

    def _process_action(self, params):
        rospy.loginfo("Dispatch processing action")
        self._action_reached("a", True, DispatchResult(True))
