#!/usr/bin/env python
# -*-coding:Utf-8 -*

import rospy
import actionlib

from geometry_msgs.msg import Pose2D
from movement_navigation_navigator.srv import Goto
from movement_navigation_navigator.msg import *

from Pathfinder import PathfinderClient
from Asserv import AsservClient

__author__ = "Gaëtan Blond"
__date__ = 17/10/2017

# Constants used for the status of goto requests
WAITING_FOR_RESULT = 0
SUCCESS = 1
FAILURE = 2

def pointToStr(point):
    """
    Convert a geometry_msgs/Pose2D object (or any object with x an y properties) to a string
    """
    return "(" + str(point.x) + "," + str(point.y) + ")"

class NavigatorNode(object):
    """
    The NavigatorNode class is the link between the AI, the Pathfinder and the Asserv.
    This node gets a movement order on ROS service. It can accept many at a time but is not design to.
    The node will wait for the Pathfinder and the Asserv to advertize their services and actions before starting itself.
    """
    def __init__ (self):
        """
        Initialize the node. Does not start it.
        """

        self._actionSrv_Dogoto = ""
        self._handledGoals = {}

        self._pathfinderClient = ""
        self._asservClient = ""
        self._waitedResults = {}
    
    def _callbackForResults(self, idAct, result):
        """
        Callback called when a goto action to asserv ended.
        It takes the id of the action and the result (DoGotoActionResult.result).
        It will only uptade the status of the request.
        @param idAct:   The id of the action.
        @param result:  The result of the action.
        """
        if result:
            self._waitedResults[idAct] = SUCCESS
        else:
            self._waitedResults[idAct] = FAILURE
    
    def _waitResult(self, idAct):
        """
        Wait for a goto (asserv) action to end.
        It will raise an exception if the action ended but did not succeed.
        @param idAct:   The id of the action.
        """
        self._waitedResults[idAct] = WAITING_FOR_RESULT
        rate = rospy.Rate(10)
        while self._waitedResults[idAct] == WAITING_FOR_RESULT:
            rate.sleep()
        
        if self._waitedResults[idAct] == FAILURE:
            raise Exception("Path found but asserv can't reach a point!")

    def _handle_goto(self, req):
        """
        Callback for the Goto service from the node.
        It will take the last position sent by the asserv as the start position.
        It will contact the Pathfinder to get the path to follow, and then sent the path point by point to the asserv.
        If something caused an abord (no path, can't move, ...) then it will return success=False.
        Else it will return success=True
        @param req:     Request containing the target position
        """
        posStart = self._asservClient.currentPose

        debugStr = "Asked to go from "
        debugStr += pointToStr(posStart)
        debugStr += " to " + pointToStr(req.targetPos)
        rospy.logdebug(debugStr)
        try:
            # sends the request to the pathfinder
            path = self._pathfinderClient.FindPath(posStart, req.targetPos)
            self._printPath (path)
            # then sends the path point per point to the arduino_asserv
            path.pop()
            for point in path:
                self._asservClient.doGoto(point, False)
            idAct = self._asservClient.doGoto(req.targetPos, True, self._callbackForResults)
            self._waitResult(idAct)
            
            # then return success
            rospy.logdebug("Success!")
            return True

        except Exception, e:
            rospy.logdebug("Navigation failled: " + e.message)
            return False

    def _printPath (self, path):
        """
        Print the path in the debug log from ROS.
        @param path:    An array of Pose2D
        """
        debugStr = "Received path: ["
        for point in path:
            debugStr += pointToStr(point)
        debugStr += "]"
        rospy.logdebug (debugStr)
    
    def _callbackAsservForDoGotoAction (self, handledGoal, resultAsserv):
        result = DoGotoResult(True)
        if not resultAsserv:
            result.success = False
        
        handledGoal.set_succeded(result)
    
    def _handleDoGotoRequest (self, handledGoal):
        posStart = self._asservClient.currentPose

        debugStr = "Asked to go from "
        debugStr += pointToStr(posStart)
        debugStr += " to " + pointToStr(handledGoal.get_goal().targetPos)
        rospy.logdebug(debugStr)

        handledGoal.set_accepted()
        id = handledGoal.status.goal_id.id
        self._handledGoals[id] = handledGoal

        try:
            # sends the request to the pathfinder
            path = self._pathfinderClient.FindPath(posStart, handledGoal.get_goal().targetPos)
            self._printPath (path)
            # then sends the path point per point to the arduino_asserv
            path.pop()
            for point in path:
                self._asservClient.doGoto(point, False)
            
            idAct = self._asservClient.doGoto(req.targetPos, True, self._callbackForResults)
            self._waitResult(idAct)

        except Exception, e:
            rospy.logdebug("Navigation failled: " + e.message)
            result = DoGotoResult(False)
            self._handledGoals[id].set_succeeded(result)

    def startNode(self):
        """
        Start the node and the clients.
        """
        rospy.init_node ('navigator_node', anonymous=False, log_level=rospy.DEBUG)

        self._pathfinderClient = PathfinderClient()
        self._asservClient = AsservClient()

        self._s = rospy.Service ("/navigation/navigator/goto", Goto, self._handle_goto)
        self._actionSrv_Dogoto = actionlib.ActionServer("/navigation/navigator/dogoto", DoGotoAction, self._handleDoGotoRequest)
        rospy.loginfo ("Ready to navigate!")
        rospy.spin ()

if __name__ == "__main__":
    try:
        node = NavigatorNode ()
        node.startNode ()
    except rospy.ROSInterruptException:
        pass
