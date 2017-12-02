#!/usr/bin/env python
# -*-coding:Utf-8 -*

import rospy
import actionlib
from actionlib.action_client import CommState

from geometry_msgs.msg import Pose2D

from drivers_ard_asserv.msg import *

from drivers_ard_asserv.srv import *

class AsservClient(object):
    def __init__ (self):
        self.ASSERV_GOTO_SERVICE_NAME = "/drivers/ard_asserv/goto"
        self.ASSERV_POSE_TOPIC_NAME = "/drivers/ard_asserv/pose2d"
        self.ASSERV_GOTOACTION_NAME = "/drivers/ard_asserv/goto_action"
        self._asservGotoService = ""
        self._asservGotoActionClient = ""

        self.currentPose = Pose2D(0.0, 0.0, 0.0)

        self._callbacksDoGoto = {}
        self._currentActions = {}
        
        self._connectToServers()
    
    def _connectToServers (self):
        rospy.loginfo('Waiting for "' + self.ASSERV_GOTO_SERVICE_NAME + '"...')
        rospy.wait_for_service(self.ASSERV_GOTO_SERVICE_NAME)
        rospy.loginfo('Asserv found.')

        # Goto service
        try:
            self._asservGotoService = rospy.ServiceProxy(self.ASSERV_GOTO_SERVICE_NAME, Goto)
            self._asservGotoActionClient = actionlib.ActionClient(self.ASSERV_GOTOACTION_NAME, DoGotoAction)
            self._asservGotoActionClient.wait_for_server()
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


    def goto (self, pos, hasAngle):
        response = False
        try:
            if hasAngle:
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
    
    def _getId(self, clientDoGotoHandle):
        str_id = clientDoGotoHandle.comm_state_machine.action_goal.goal_id.id
        idAct = int(str_id.split('-')[1])
        rospy.logdebug("ID: " + str(idAct))
        return idAct
    
    def doGoto (self, pos, hasAngle=False, callback=None):
        mode = DoGotoGoal.GOTO
        if hasAngle:
            mode = DoGotoGoal.GOTOA
        goal = DoGotoGoal(mode=mode, position=pos)
        goalHandle = self._asservGotoActionClient.send_goal(goal, transition_cb=self._handleDoGotoResult)
        idAct = self._getId(goalHandle)

        if callback:
            self._callbacksDoGoto[idAct] = callback
        
        self._currentActions[idAct] = goalHandle

        return idAct
    
    def _handleDoGotoResult (self, clientDoGotoHandle):
        if clientDoGotoHandle.get_comm_state() == CommState.DONE:
            rospy.logdebug("DONE")
            idAct = self._getId(clientDoGotoHandle)
            if idAct in self._callbacksDoGoto:
                self._callbacksDoGoto[idAct](idAct, clientDoGotoHandle.get_result().result)
                del self._callbacksDoGoto[idAct]
                del self._currentActions[idAct]