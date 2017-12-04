#!/usr/bin/python
import rospy

from ai_game_status.msg import GameEnd
from ai_game_status.srv import ArmRequest, NodeReady


class StatusServices(object):
    READY_SRV = "/ai/game_status/ready_notification"
    HALT_SRV  = "/ai/game_status/status"

    def __init__(self, namespace, packagename, arm_cb = None, halt_cb = None):
        self._namespace, self._packagename = namespace, packagename
        if arm_cb:
            self._arm_srv   = rospy.Service("/{}/{}/arm".format(namespace, packagename), ArmRequest, arm_cb)
        if halt_cb:
            self._halt_srv  = rospy.Subscriber(self.HALT_SRV, GameEnd, halt_cb)

    def ready(self, success):
        rospy.wait_for_service(self.READY_SRV)
        _ready_pub = rospy.ServiceProxy(self.READY_SRV, NodeReady)
        _ready_pub("/{}/{}".format(self._namespace, self._packagename), success)
