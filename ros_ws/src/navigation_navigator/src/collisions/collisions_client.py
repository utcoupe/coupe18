#!/usr/bin/env python
# -*-coding:Utf-8 -*

import rospy
from navigation_collisions.msg import PredictedCollision

__author__ = "Gaëtan Blond"
__date__ = 11/12/2017

COLLISIONS_WATCHDOG_TIME = 0.5


class CollisionsClient(object):
    def __init__ (self, callbackStop, callbackResume):
        self.WARNER_TOPIC = "/navigation/collisions/warner"
        self._callbackStop = callbackStop
        self._callbackResume = callbackResume
        self._tmr_collisions_check = rospy.Timer(rospy.Duration(COLLISIONS_WATCHDOG_TIME), self._callback_timer_collisions_watchdog)
        self._last_collision = False
        self._collision_active = False
        self._connectToServers()

    def _warnerCallback (self, message):
        if message.danger_level == message.LEVEL_DANGER:
            self._last_collision = True
            self._collision_active = True
            rospy.loginfo("Obstacle detected, stopping the robot")
            self._callbackStop()

    def _connectToServers (self):
        rospy.loginfo("Waiting for \"" + self.WARNER_TOPIC + "\"")
        self._wait_for_topic(self.WARNER_TOPIC)
        rospy.loginfo("Collisions found")

        try:
            rospy.Subscriber(self.WARNER_TOPIC, PredictedCollision, self._warnerCallback)
        except rospy.ServiceException, e:
            str_error = "Error when trying to connect to "
            str_error += self.WARNER_TOPIC
            str_error += " : " + str(e)
            rospy.logerr(str_error)
    
    def _wait_for_topic(self, topic):
        found = False
        rate = rospy.Rate(5)
        while not found:
            found = topic in dict(rospy.get_published_topics()).keys()
            rate.sleep()

    def _callback_timer_collisions_watchdog(self, event):
        if self._collision_active:
            if not self._last_collision:
                self._callbackResume()
                self._collision_active = False
            self._last_collision = False
