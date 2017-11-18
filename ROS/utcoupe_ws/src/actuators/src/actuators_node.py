#!/usr/bin/python2.7
# -*-coding:Utf-8 -*

import rospy
import actionlib
import actuators.msg


class ActuatorsNode():
    """Dispatch commands from AI to the correct node"""

    _result = actuators.msg.dispatchResult()

    def __init__(self):
        self._node = rospy.init_node('actuators')
        self._action_name = 'dispatcher'
        self._action_server = actionlib.SimpleActionServer(self._action_name, actuators.msg.dispatchAction, execute_cb=self.dispatch, auto_start = False)
        self._action_server.start()

    def dispatch(self, command):
        #rospy.loginfo('Command received !')
        self._action_server.set_succeeded(False)

if __name__ == '__main__':
    actuators_node = ActuatorsNode()
    rospy.loginfo('Actuators node started')
    rospy.spin()
