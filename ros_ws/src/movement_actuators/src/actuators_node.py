#!/usr/bin/python2.7
# -*-coding:Utf-8 -*

import rospy
import actionlib
import actuators.msg

class ActuatorsNode():
    """Dispatch commands from AI to the correct node"""

    _result = actuators.msg.dispatchResult()

    def __init__(self):
        self._node = rospy.init_node('actuators', log_level=rospy.DEBUG)
        self._namespace = '/movement/actuators/'
        self._action_name = '{}dispatch'.format(self._namespace)
        self._action_server = actionlib.SimpleActionServer(self._action_name, actuators.msg.dispatchAction, execute_cb=self.dispatch, auto_start = False)
        
        # TODO: rename arduino
        self._arduino_client = actionlib.SimpleActionClient(
            '{}arduino'.format(self._namespace), actuators.msg.arduinoAction)
        self._action_server.start()

    def dispatch(self, command):
        rospy.logdebug('Command received !')
        if command.name != '':
            pass

        self._action_server.set_succeeded(False)

    def sendToArduino(self, ):
        _arduino_client.wait_for_server()
        goal = actuators.msg.arduinoGoal(order=20)
        _arduino_client.send_goal(goal)
        _arduino_client.wait_for_result()
        _arduino_client.get_result()

if __name__ == '__main__':
    actuators_node = ActuatorsNode()
    rospy.loginfo('Actuators node started')
    rospy.spin()
