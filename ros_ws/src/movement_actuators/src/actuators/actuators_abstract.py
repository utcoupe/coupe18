#!/usr/bin/env python

import rospy
import actionlib

__author__ = "Thomas Fuhrmann"
__date__ = 9/04/2018

NODE_NAME = "actuators"


class ActuatorsAbstract:
    def __init__(self, action_name, action_type):
        # TODO manage system HALT with game_status stuff (using main node ?)
        # Dictionary to manage received goals (mandatory for actionlib)
        self._goals_handler_dictionary = {}
        self._action_type = None
        if action_name != "" and action_type is not None:
            self._action_type = action_type
            self._action_server = actionlib.ActionServer("/movement/" + NODE_NAME + "/" + action_name, action_type, self._callback_action, auto_start=False)
            self._action_server.start()
        else:
            rospy.logerr("ActuatorsAbstract class can not be built without action_name and action_type, aborting.")
            exit(1)

    def _action_reached(self, goal_id, reached, result):
        # Result is passed as parameter to avoid retrieving the goal result type in abstract class, maybe a TODO ?
        if reached:
            rospy.loginfo("Goal id {}, has been reached.".format(goal_id))
        else:
            rospy.logwarn("Goal id {}, has NOT been reached.".format(goal_id))
        if goal_id in self._goals_handler_dictionary:
            self._goals_handler_dictionary[goal_id].set_succeeded(result)
            del self._goals_handler_dictionary[goal_id]

    def _process_action(self, goal):
        rospy.logerr("ActuatorsAbstract is abstract !")
        return False

    def _callback_action(self, goal_handle):
        rospy.logdebug("Just received an action")
        goal = goal_handle.get_goal()

        # TODO manage goal rejection if node in bad mode

        if self._process_action(goal_handle.get_goal()):
            goal_handle.set_accepted()
            self._goals_handler_dictionary[goal_handle.get_goal_id()] = goal_handle
        else:
            goal_handle.set_rejected()
