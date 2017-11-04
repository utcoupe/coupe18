#!/usr/bin/env python
# -*-coding:Utf-8 -*

import rospy

from asserv.msg import Goto

class AsservClient:
    def __init__ (self):
        self.ASSERV_GOTO_SERVICE_NAME = "asserv/controls/goto"
        self.asservGotoService = ""
    
    def _ConnectToServer (self):
        rospy.loginfo('Waiting for "' + self.ASSERV_GOTO_SERVICE_NAME + '"...')
        rospy.wait_for_service(self.ASSERV_GOTO_SERVICE_NAME)
        rospy.loginfo('Asserv found.')
        try:
            self.asservGotoService = rospy.ServiceProxy(self.ASSERV_GOTO_SERVICE_NAME, Goto)
        except rospy.ServiceException, e:
            error_str = "Error when trying to connect to "
            error_str += self.ASSERV_GOTO_SERVICE_NAME
            error_str += " : " + str(e)
            rospy.logfatal (error_str)
    
    def Goto (self, point):
        response = False
        try:
            response = self.asservGotoService(int(point.x), int(point.y), 0.0)
        except rospy.ServiceException, e:
            error_str = "Error when trying to use "
            error_str += self.ASSERV_GOTO_SERVICE_NAME
            error_str += " : " + str(e)
            rospy.logerr (error_str)
            raise Exception
        else:
            if not response:
                raise Exception