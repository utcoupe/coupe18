#!/usr/bin/env python

import rospy
import actionlib
from drivers_ax12.msg import *

class Ax12:
    def __init__(self):
        rospy.init_node('ax12', anonymous=False)

        self.current_goals = {}
        self.next_goal_id = 0

        self._act_goto = actionlib.ActionServer("ax12/command_action", AngleCommandAction,
                                                self.action_callback, auto_start=False)
        self._act_goto.start()

    def action_callback(self, goal_handled):
        goal = goal_handled.get_goal()

