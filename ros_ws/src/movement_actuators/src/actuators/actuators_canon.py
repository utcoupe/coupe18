#!/usr/bin/env python

import rospy
import actionlib
from movement_actuators.srv import *
from movement_actuators.msg import DispatchAction, DispatchGoal

__author__ = "Thomas Fuhrmann"
__date__ = 1/05/2018

DISPATCH_SERVER_TIMEOUT = 10  # in seconds
FLIPPER_PERIOD = 1  # in seconds
FLIPPER_NAME_CANON = "servo_flipper_side_canon"
FLIPPER_NAME_BIN = "servo_flipper_side_bin"
CANON_NAME = "canon"
DEFAULT_ACTUATORS_TIMEOUT = 1000


class ActuatorsCanon:
    def __init__(self):
        self._client = actionlib.SimpleActionClient('/movement/actuators/dispatch', DispatchAction)
        self._client.wait_for_server(rospy.Duration(DISPATCH_SERVER_TIMEOUT))
        self._tmr_flipper_activate = False
        self._tmr_flipper = rospy.Timer(rospy.Duration(FLIPPER_PERIOD), self._callback_timer_flipper)
        self._srv_canon = rospy.Service("/movement/actuators/activate_canon", ActivateCanon, self._callback_activate_canon)
        # Boolean used for flipper toggle values, as they just flip on 2 values
        self._flipper_state = False

    def _callback_activate_canon(self, request):
        response = True
        if request.fire_distance == -1:
            self._flipper_stop()
            self._canon_stop()
        elif request.fire_distance >= 0:
            self._flipper_start()
            self._canon_start(request.fire_distance)
        else:
            response = False
        return ActivateCanonResponse(response)

    def _flipper_start(self):
        self._tmr_flipper_activate = True

    def _flipper_stop(self):
        self._tmr_flipper_activate = False

    def _canon_start(self, distance):
        recv_distance = distance
        if recv_distance == 0:
            rospy.logwarn("Automatic distance computation is not implemented yet. Use default value (1m)")
            recv_distance = 1
        ref_value = 16*recv_distance + 32
        rospy.loginfo("Computed ref value : " + str(ref_value))
        goal_canon = DispatchGoal()
        goal_canon.name = CANON_NAME
        goal_canon.param = str(int(ref_value))
        goal_canon.timeout = DEFAULT_ACTUATORS_TIMEOUT
        self._client.send_goal(goal_canon)

    def _canon_stop(self):
        goal_canon = DispatchGoal()
        goal_canon.name = CANON_NAME
        goal_canon.preset = "OFF"
        goal_canon.timeout = DEFAULT_ACTUATORS_TIMEOUT
        self._client.send_goal(goal_canon)

    def _callback_timer_flipper(self, event):
        if self._tmr_flipper_activate:
            if self._flipper_state:
                preset = "MIN"
                self._flipper_state = False
            else:
                preset = "MAX"
                self._flipper_state = True
            goal_flipper_canon = DispatchGoal()
            goal_flipper_canon.name = FLIPPER_NAME_CANON
            goal_flipper_canon.preset = preset
            goal_flipper_canon.timeout = DEFAULT_ACTUATORS_TIMEOUT
            self._client.send_goal(goal_flipper_canon)
            goal_flipper_bin = DispatchGoal()
            goal_flipper_bin.name = FLIPPER_NAME_BIN
            goal_flipper_bin.preset = preset
            goal_flipper_bin.timeout = DEFAULT_ACTUATORS_TIMEOUT
            self._client.send_goal(goal_flipper_bin)
