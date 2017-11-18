#!/usr/bin/python2.7
# -*-coding:Utf-8 -*

import rospy
import actionlib
import movement_actuators.msg
import actuators_properties

class ActuatorsNode():
    """Dispatch commands from AI to the correct node"""

    _result = movement_actuators.msg.dispatchResult()

    def __init__(self):
        self._node = rospy.init_node('actuators', log_level=rospy.DEBUG)
        self._namespace = '/movement/actuators/'
        self._action_name = '{}dispatch'.format(self._namespace)
        self._action_server = actionlib.SimpleActionServer(
            self._action_name, movement_actuators.msg.dispatchAction, execute_cb=self.dispatch, auto_start=False)
        
        # TODO: rename arduino
        self._arduino_client = actionlib.SimpleActionClient(
            '{}arduino'.format(self._namespace), movement_actuators.msg.arduinoAction)
        self._action_server.start()

    def dispatch(self, command):
        rospy.logdebug('Command received !')
        #-----Actuator check
        try:
            actuator = actuators_properties.getActuatorsList()[command.name]
        except KeyError as identifier:
            rospy.logwarn('The action dispatch should be called with a name instead of an id.')
            for actuator in actuators_properties.getActuatorsList().values():
                if actuator.id==command.id:
                    break
                actuator = None
            if actuator == None:
                self._action_server.set_succeeded(False)
                rospy.logerr('Couldn\'t find actuators with name "' +
                             command.name + '" or id "' + str(command.id)+'".')
                return
        #-----Param check
        param = None
        if command.param == '':
            try:
                param = actuator.preset[command.preset]
            except KeyError as identifier:
                self._action_server.set_succeeded(False)
                rospy.logerr('Couldn\'t find preset with name "' +
                             command.preset + '" in actuator "' + actuator.name + '"')
                return
        else:
            param = command.param
        #-----Timeout check
        timeout = None
        if command.timeout==-1:
            timeout = actuator.default_timeout
        else:
            timeout = command.timeout
        #-----Time to send !
        if actuator.family == 'arduino':
            # self._action_server.set_succeeded(self.sendToArduino(actuator.id, command.order, param, timeout))
            self._action_server.set_succeeded(False)
            return
        elif actuator.family == 'ax12':
            self._action_server.set_succeeded(sendToAx12(
                actuator.id, command.order, param, timeout))
            return 

        self._action_server.set_succeeded(False)

    def sendToArduino(self, id, order, param, timeout):
        _arduino_client.wait_for_server()
        goal = actuators.msg.arduinoGoal(id=id, order=order, param=param)
        _arduino_client.send_goal(goal)
        _arduino_client.wait_for_result(rospy.Duration(0, timeout * 1000))
        return _arduino_client.get_result().success

    def sendToAx12(self, id, order, param, timeout):
        return False


if __name__ == '__main__':
    actuators_node = ActuatorsNode()
    rospy.loginfo('Actuators node started')
    rospy.spin()
