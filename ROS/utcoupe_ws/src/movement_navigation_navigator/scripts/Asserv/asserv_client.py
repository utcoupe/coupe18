#!/usr/bin/env python
# -*-coding:Utf-8 -*

import rospy

from geometry_msgs.msg import Pose2D

from asserv.msg import DoGotoAction

from asserv.srv import *

class AsservClient:
    def __init__ (self):
        self.ASSERV_GOTO_SERVICE_NAME = "asserv/controls/goto"
        self.ASSERV_POSE_TOPIC_NAME = "robot/pose2d"
        self._asservGotoService = ""

        self.currentPose = Pose2D(0.0, 0.0, 0.0)
        
        self._connectToServers()
    
    def _connectToServers (self):
        rospy.loginfo('Waiting for "' + self.ASSERV_GOTO_SERVICE_NAME + '"...')
        rospy.wait_for_service(self.ASSERV_GOTO_SERVICE_NAME)
        rospy.loginfo('Asserv found.')

        # Goto service
        try:
            self._asservGotoService = rospy.ServiceProxy(self.ASSERV_GOTO_SERVICE_NAME, Goto)
        except rospy.ServiceException, e:
            error_str = "Error when trying to connect to "
            error_str += self.ASSERV_GOTO_SERVICE_NAME
            error_str += " : " + str(e)
            rospy.logfatal (error_str)
        
        # Pose topic
        try:
            rospy.Subscriber(self.ASSERV_POSE_TOPIC_NAME, Pose2D, self._handleUpdatePose)
        except:
            pass

    def _handleUpdatePose (self, newPose):
        self.currentPose = newPose


    def goto (self, pos, angle):
        response = False
        try:
            if angle:
                response = self._asservGotoService(mode=GotoRequest.GOTOA, position=pos).response
            else:
                response = self._asservGotoService(mode=GotoRequest.GOTO, position=pos).response
        except rospy.ServiceException, e:
            error_str = "Error when trying to use "
            error_str += self.ASSERV_GOTO_SERVICE_NAME
            error_str += " : " + str(e)
            rospy.logerr (error_str)
            raise Exception
        else:
            if not response:
                raise Exception("Path valid but can't reach a point.")