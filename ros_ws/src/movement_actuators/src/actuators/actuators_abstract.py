#!/usr/bin/env python

import rospy
import actionlib

__author__ = "Thomas Fuhrmann"
__date__ = 9/04/2018

NODE_NAME = "actuators"

class ActuatorsAbstract:
    def __init__(self, action_name, action_type):
        # Dictionary to manage received goals (mandatory for actionlib)
        self._goals_dictionary = {}
        self._action_type = None
        self._tmp_goal = None
        if action_name != "" and action_type is not None:
            self._action_type = action_type
            self._action_server = actionlib.ActionServer("/movement/" + NODE_NAME + "/" + action_name, action_type, self._callback_action, auto_start=False)
            self._action_server.start()
        else:
            rospy.logerr("ActuatorsAbstract class can not be built whithout action_name and action_type, aborting.")
            exit(1)

    def _action_reached(self, action_id, reached, result):
        if reached:
            rospy.loginfo("Goal id {}, has been reached.".format(action_id))
        else:
            rospy.logwarn("Goal id {}, has NOT been reached.".format(action_id))
        self._tmp_goal.set_succeeded(result)
        # result = DoGotoResult(reached)
        # if goal_id in self._goals_dictionary:
        #     self._goals_dictionary[goal_id].set_succeeded(result)
        #     del self._goals_dictionary[goal_id]
        # elif goal_id in self._goto_srv_dictionary:
        #     del self._goto_srv_dictionary[goal_id]

    def _process_action(self, params):
        rospy.logerr("ActuatorsAbstract is abstract !")
        return False

    def _callback_action(self, goal_handle):
        rospy.logdebug("Just received an action")
        goal_handle.set_accepted()
        self._tmp_goal = goal_handle
        self._process_action("")
