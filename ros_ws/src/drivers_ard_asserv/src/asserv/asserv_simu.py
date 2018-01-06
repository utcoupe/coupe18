#!/usr/bin/env python

import rospy
from asserv_abstract import *

__author__ = "Thomas Fuhrmann"
__date__ = 16/12/2017


class AsservSimu(AsservAbstract):
    def __init__(self, asserv_node):
        AsservAbstract.__init__(self, asserv_node)
        rospy.loginfo("AsservSimu")

    def goto(self, goal_id, x, y, direction):
        rospy.logerr("AsservAbstract is abstract !")
        return False

    def gotoa(self, goal_id, x, y, a, direction):
        rospy.logerr("AsservAbstract is abstract !")
        return False

    def rot(self, goal_id, a, no_modulo):
        rospy.logerr("AsservAbstract is abstract !")
        return False

    def pwm(self, left, right, duration):
        rospy.logerr("AsservAbstract is abstract !")
        return False

    def speed(self, linear, angular, duration):
        rospy.logerr("AsservAbstract is abstract !")
        return False

    def set_emergency_stop(self, stop):
        rospy.logerr("AsservAbstract is abstract !")
        return False

    def kill_goal(self):
        rospy.logerr("AsservAbstract is abstract !")
        return False

    def clean_goals(self):
        rospy.logerr("AsservAbstract is abstract !")
        return False

    def pause(self, pause):
        rospy.logerr("AsservAbstract is abstract !")
        return False

    def reset_id(self):
        rospy.logerr("AsservAbstract is abstract !")
        return False

    def set_max_speed(self, speed, speed_ratio):
        rospy.logerr("AsservAbstract is abstract !")
        return False

    def set_max_accel(self, accel):
        rospy.logerr("AsservAbstract is abstract !")
        return False

    def set_pid(self, p, i, d):
        rospy.logerr("AsservAbstract is abstract !")
        return False

    def set_pos(self, x, y, a):
        rospy.logerr("Simu setpos !")
        return True
