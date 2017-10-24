#!/usr/bin/env python
# -*-coding:Utf-8 -*

# reçoit ordre de ai deplacement de posinit à posfin
#--> demande chemin pathfinding
#----> si impossible retourne impossible à ai
#--> envoi chemin à arduino_asserv point par point
#----> si impossible retourne impossible à ai
#--> retourner travail_fini à ai

import rospy

from geometry_msgs.msg import Point
from movement_navigation_navigator.srv import Goto

from Pathfinder import PathfinderClient

def pointToStr(point):
    return "(" + str(point.x) + "," + str(point.y) + ")"

class NavigatorNode:
    def __init__ (self):
        self._pathfinderClient = PathfinderClient()

    def _handle_goto(self, req):
        debugStr = "Asked to go from "
        debugStr += pointToStr(req.posStart)
        debugStr += " to " + pointToStr(req.posEnd)
        rospy.logdebug(debugStr)
        # sends the request to the pathfinder
        path = self._pathfinderClient.FindPath(req.posStart, req.posEnd)
        self._printPath (path)
        # then sends the path point per point to the arduino_asserv
        # TODO
        # then return success
        return True

    def _printPath (self, path):
        debugStr = "Received path: ["
        for point in path:
            debugStr += pointToStr(point)
        debugStr += "]"
        rospy.logdebug (debugStr)

    def startNode(self):
        rospy.init_node ('navigator')

        self._s = rospy.Service ("/navigation/navigator/goto", Goto, self._handle_goto)
        rospy.loginfo ("Ready to navigate!")
        rospy.spin ()

if __name__ == "__main__":
    try:
        node = NavigatorNode ()
        node.startNode ()
    except rospy.ROSInterruptException:
        pass
