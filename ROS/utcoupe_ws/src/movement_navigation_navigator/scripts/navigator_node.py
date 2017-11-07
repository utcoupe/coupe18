#!/usr/bin/env python
# -*-coding:Utf-8 -*

import rospy

from geometry_msgs.msg import Pose2D
from movement_navigation_navigator.srv import Goto

from Pathfinder import PathfinderClient
from Asserv import AsservClient

def pointToStr(point):
    return "(" + str(point.x) + "," + str(point.y) + ")"

class NavigatorNode:
    def __init__ (self):
        self._pathfinderClient = ""
        self._asservClient = ""
        self._waitedResults = {}
    
    def _callbackForResults(id):
        self._waitedResults[id] = False

    def _handle_goto(self, req):
        #posStart = self._asservClient.currentPose
        posStart = Pose2D(1.0,1.0,0.0)

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
            id = self._asservClient.doGoto(req.targetPos, True, self._callbackForResults)
            self._waitedResults[id] = True
            rate = rospy.Rate(10)
            while self._waitedResults[id]:
                rate.sleep()
            
            # then return success
            rospy.logdebug("Success!")
            return True

        except Exception, e:
            rospy.logdebug("Navigation failled: " + e.message)
            return False

    def _printPath (self, path):
        debugStr = "Received path: ["
        for point in path:
            debugStr += pointToStr(point)
        debugStr += "]"
        rospy.logdebug (debugStr)

    def startNode(self):
        rospy.init_node ('navigator')

        self._pathfinderClient = PathfinderClient()
        self._asservClient = AsservClient()

        self._s = rospy.Service ("/navigation/navigator/goto", Goto, self._handle_goto)
        rospy.loginfo ("Ready to navigate!")
        rospy.spin ()

if __name__ == "__main__":
    try:
        node = NavigatorNode ()
        node.startNode ()
    except rospy.ROSInterruptException:
        pass
