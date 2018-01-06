#!/usr/bin/env python
# -*-coding:Utf-8 -*

import rospy
from navigation_collisions.msg import PredictedCollision

__author__ = "Gaëtan Blond"
__date__ = 11/12/2017

class CollisionsClient(object):
    def __init__ (self, callbackStop):
        self.WARNER_TOPIC = "/navigation/collisions/warner"

        self._callbackStop = callbackStop

        self._connectToServers()


    
    def _warnerCallback (self, message):
        if message.danger_level == message.LEVEL_STOP:
            rospy.logdebug("Obstacle detected, stopping the robot")
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