#!/usr/bin/python2.7
# -*-coding:Utf-8 -*

import rospy
import actuators
from ai_game_status import StatusServices

__author__ = "P. Potiron", "Thomas Fuhrmann"
__date__ = 9/04/2018

NODE_NAME = "actuators"


class ActuatorsNode:
    def __init__(self):
        rospy.init_node(NODE_NAME, log_level=rospy.INFO)
        self.dispatch_instance = actuators.ActuatorsDispatch()

        robot = rospy.get_param('/robot')
        if robot.lower() == "gr":
            self.arm_instance = actuators.ActuatorsArm()
        elif robot.lower() == "pr":
            self.barrel_instance = actuators.ActuatorsBarrel()

        rospy.loginfo("Movement actuators has correctly started.")
        StatusServices("movement", "actuators", None, self._on_gameStatus).ready(True)
        rospy.spin()
    
    def _on_gameStatus(self, msg):
        pass


if __name__ == '__main__':
    ActuatorsNode()
