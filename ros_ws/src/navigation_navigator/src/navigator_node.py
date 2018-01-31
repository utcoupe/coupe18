#!/usr/bin/env python
# -*-coding:Utf-8 -*

from functools import partial

import rospy
import actionlib

from geometry_msgs.msg import Pose2D
from navigation_navigator.srv import Goto
from navigation_navigator.msg import Status, DoGotoResult, DoGotoAction

from pathfinder import PathfinderClient
from asserv import AsservClient
from localizer import LocalizerClient
from collisions import CollisionsClient

from ai_game_status import StatusServices


__author__ = "Gaëtan Blond"
__date__ = 17/10/2017

NODE_NAME = "navigator"
FULL_NODE_NAME = "/navigation/" + NODE_NAME

# Constants used for the status of goto requests
class GotoStatuses(object):
    WAITING_FOR_RESULT = 0
    SUCCESS = 1
    FAILURE = 2

# Constants used for the status topic
class NavigatorStatuses(object):
    NAV_IDLE = 0
    NAV_NAVIGATING = 1
    NAV_STOPPED = 2

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
        self._gotoSrv = ""
        self._statusPublisher = ""

        self._pathfinderClient = ""
        self._asservClient = ""
        self._localizerClient = ""
        self._collisionsClient = ""
        self._waitedResults = {}

        self._currentStatus = NavigatorStatuses.NAV_IDLE
        self._currentPath = {}

        # Tell ai/game_status the node initialized successfuly.
        StatusServices("navigation", "navigator").ready(True)

    def _callbackForResults(self, idAct, result):
        """
        Callback called when a goto action to asserv ended.
        It takes the id of the action and the result (DoGotoActionResult.result).
        It will only uptade the status of the request.
        @param idAct:   The id of the action.
        @param result:  The result of the action.
        """
        if result:
            self._waitedResults[idAct] = GotoStatuses.SUCCESS
        else:
            self._waitedResults[idAct] = GotoStatuses.FAILURE

    def _waitResult(self, idAct):
        """
        Wait for a goto (asserv) action to end.
        It will raise an exception if the action ended but did not succeed.
        @param idAct:   The id of the action.
        """
        self._waitedResults[idAct] = GotoStatuses.WAITING_FOR_RESULT
        rate = rospy.Rate(10)
        while self._waitedResults[idAct] == GotoStatuses.WAITING_FOR_RESULT:
            rate.sleep()

        if self._waitedResults[idAct] == GotoStatuses.FAILURE:
            raise Exception("Path found but asserv can't reach a point!")

    def _handle_goto(self, req):
        """
        Callback for the Goto service from the node.
        It will take the last position sent by the localizer as the start position.
        It will contact the Pathfinder to get the path to follow, and then sent the path point by
        point to the asserv.
        If something caused an abord (no path, can't move, ...) then it will return success=False.
        Else it will return success=True
        @param req:     Request containing the target position
        """
        posStart = self._localizerClient.getLastKnownPos()
        debugStr = "Asked to go from "
        debugStr += pointToStr(posStart)
        debugStr += " to " + pointToStr(req.target_pos)
        rospy.logdebug(debugStr)
        try:
            # sends the request to the pathfinder
            path = self._pathfinderClient.FindPath(posStart, req.target_pos)
            self._printPath(path)
            # then sends the path point per point to the arduino_asserv
            path.pop()
            for point in path:
                self._asservClient.doGoto(point, False)
            idAct = self._asservClient.doGoto(req.target_pos, True, self._callbackForResults)
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

    def _callbackAsservForDoGotoAction (self, handledGoal, isFinalPos, idAct, resultAsserv):
        if self._currentPath:
            self._currentPath.pop(0)
        self._updateStatus()
        if isFinalPos:
            result = DoGotoResult(True)
            if (not resultAsserv) or (self._currentStatus == NavigatorStatuses.NAV_STOPPED):
                result.success = False
            self._currentStatus = NavigatorStatuses.NAV_IDLE
            self._collisionsClient.setEnabled(False)
            handledGoal.set_succeeded(result)

    def _handleDoGotoRequest (self, handledGoal):
        """
        Callback for the navigator's goto action request.
        The start position is the last received one from the localizer.
        If there are no path between start and end positions, it will respond with success=False.
        Else all waypoints are send to the asserv and the navigator will respond when the asserv
        will have treated all points.
        @param handledGoal: the received goal
        """
        self._currentStatus = NavigatorStatuses.NAV_NAVIGATING
        self._collisionsClient.setEnabled(not handledGoal.get_goal().disable_collisions)
        posStart = self._localizerClient.getLastKnownPos()
        posEnd = handledGoal.get_goal().target_pos
        debugStr = "Asked to go from "
        debugStr += pointToStr(posStart)
        debugStr += " to " + pointToStr(posEnd)
        rospy.logdebug(debugStr)
        handledGoal.set_accepted()
        try:
            # sends the request to the pathfinder
            path = self._pathfinderClient.FindPath(posStart, posEnd)
            self._printPath (path)
            # then sends the path point per point to the arduino_asserv
            path.pop() # The last point will be given endPosition
            self._currentPath = path[:]
            self._currentPath.append(posEnd)
            self._updateStatus()
            for point in path:
                cb = partial(self._callbackAsservForDoGotoAction, handledGoal, False)
                self._asservClient.doGoto(point, False, cb)
            
            hasAngle = False
            if handledGoal.get_goal().mode == handledGoal.get_goal().GOTOA:
                hasAngle = True
            cb = partial(self._callbackAsservForDoGotoAction, handledGoal, True)
            self._asservClient.doGoto(posEnd, hasAngle, cb)
        except Exception, e:
            rospy.logdebug("Navigation failed: " + e.message)
            result = DoGotoResult(False)
            self._currentStatus = NavigatorStatuses.NAV_IDLE
            self._collisionsClient.setEnabled(False)
            self._updateStatus()
            handledGoal.set_succeeded(result)

    def _callbackEmergencyStop (self):
        """
        Ask the asserv to stop and update the status
        """
        self._currentStatus = NavigatorStatuses.NAV_STOPPED
        self._asservClient.stopAsserv()
        self._updateStatus()

    def _callbackAsservResume(self):
        self._currentStatus = NavigatorStatuses.NAV_NAVIGATING
        self._collisionsClient.setEnabled(True)
        self._asservClient.resumeAsserv()
        self._updateStatus()

    def _updateStatus (self):
        """
        Send the current status and waypoint list in the navigator's status topic.
        """
        statusMsg = Status()
        statusMsg.status = self._currentStatus
        if len(self._currentPath) > 0:
            statusMsg.currentPath = self._currentPath
        self._statusPublisher.publish(statusMsg)


    def startNode(self):
        """
        Start the node and the clients.
        """
        rospy.init_node (NODE_NAME, anonymous=False, log_level=rospy.INFO)
        # Create the clients
        self._pathfinderClient = PathfinderClient()
        self._asservClient = AsservClient()
        self._localizerClient = LocalizerClient()
        self._collisionsClient = CollisionsClient(self._callbackEmergencyStop, self._callbackAsservResume)
        # Create service server, action server and topic publisher
        self._gotoSrv = rospy.Service (FULL_NODE_NAME + "/goto", Goto, self._handle_goto)
        self._actionSrv_Dogoto = actionlib.ActionServer(FULL_NODE_NAME + "/goto_action", DoGotoAction, self._handleDoGotoRequest, auto_start=False)
        self._statusPublisher = rospy.Publisher(FULL_NODE_NAME + "/status", Status, queue_size=10)
        # Launch the node
        self._actionSrv_Dogoto.start()
        rospy.loginfo ("Ready to navigate!")
        self._updateStatus()
        rospy.spin ()

if __name__ == "__main__":
    try:
        node = NavigatorNode ()
        node.startNode ()
    except rospy.ROSInterruptException:
        pass
