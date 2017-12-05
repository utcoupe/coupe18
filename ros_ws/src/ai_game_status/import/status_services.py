#!/usr/bin/python
import rospy

from ai_game_status.msg import GameStatus
from ai_game_status.srv import ArmRequest, ArmRequestResponse, NodeReady


class StatusServices(object):
    READY_SRV = "/ai/game_status/node_ready" # Service to call when the node has finished its init phase (successful or not).
    ARM_SRV   = "/arm"                       # Server the node can use if it needs to be calibrated at one point (called by scheduler before jack)
    HALT_SRV  = "/ai/game_status/status"     # Topic that can be used to know when HALT is activated (if the node needs to be stopped).

    def __init__(self, namespace, packagename, arm_cb = None, status_cb = None):
        self.node_name = "/{}/{}".format(namespace, packagename)
        self.arm_cb = arm_cb
        if arm_cb: rospy.Service(self.node_name + self.ARM_SRV, ArmRequest, self._on_arm)
        if status_cb: rospy.Subscriber(self.HALT_SRV, GameStatus, status_cb)

    def ready(self, success):
        rospy.wait_for_service(self.READY_SRV, timeout = 4.0)
        _ready_pub = rospy.ServiceProxy(self.READY_SRV, NodeReady)
        _ready_pub(self.node_name, success)

    def _on_arm(self, req): # Handle Service response here.
        success = self.arm_cb()
        return ArmRequestResponse(success)
