#!/usr/bin/env python

import rospy
import time
import random
import threading
from actuators_abstract import *
import actuators_properties
from movement_actuators.msg import DispatchAction, DispatchResult
import drivers_ard_others.msg
import drivers_ax12.msg

__author__ = "P. Potiron", "Thomas Fuhrmann"
__date__ = 9/04/2018


def current_milli_time(): return int(round(time.time() * 1000))

class ActuatorsDispatch(ActuatorsAbstract):

    _result = DispatchResult()

    def __init__(self):
        ActuatorsAbstract.__init__(self, "dispatch", DispatchAction)
        self._lock = threading.RLock()
        # Random_id, thread_event
        self._call_stack = {}
        self._pub_ard_move = rospy.Publisher('/drivers/ard_others/move', drivers_ard_others.msg.Move, queue_size=3)
        self._sub_ard_response = rospy.Subscriber('/drivers/ard_others/move_response', drivers_ard_others.msg.MoveResponse, self._callback_move_response)
        self._act_cli_ax12 = actionlib.SimpleActionClient('/drivers/ax12', drivers_ax12.msg.Ax12CommandAction)
        rospy.loginfo("Dispatch constructed")

    def _process_action(self, goal):
        rospy.loginfo("Dispatch processing action")
        # self._action_reached("a", True, DispatchResult(True))
        actuator = None
        # -----Actuator check
        try:
            actuator_list = actuators_properties.getActuatorsList()
            if actuator_list is not None:
                actuator = actuator_list[goal.name]
            else:
                actuator = None
        except KeyError as identifier:
            rospy.logwarn('The action dispatch should be called with a name instead of an id.')
            for actuator in actuators_properties.getActuatorsList().values():
                if actuator.id == goal.id:
                    break
                actuator = None
        if actuator is None:
            self._action_reached(goal.id, False, DispatchResult(False))
            rospy.logerr('Couldn\'t find actuators with name "' +
                         goal.name + '" or id "' + str(goal.id) + '".')
            return
        # -----Param check
        param = None
        if goal.param == '':
            try:
                param_list = actuator.preset
                if param_list is not None:
                    param = param_list[goal.preset]
                else:
                    param = None
            except KeyError as identifier:
                self._action_reached(goal.id, False, DispatchResult(False))
                rospy.logerr('Couldn\'t find preset with name "' +
                             goal.preset + '" in actuator "' + actuator.name + '"')
                return
        else:
            param = int(goal.param)  # TODO check param
        # -----Timeout check
        timeout = None
        if goal.timeout <= 0:
            timeout = actuator.default_timeout
        else:
            timeout = goal.timeout
        # -----Time to send !
        if actuator.family.lower() == 'arduino':
            self._result.success = self.sendToArduino(actuator.id, actuator.type, goal.order, param, timeout)
            self._action_server.set_succeeded(self._result)
            return
        elif actuator.family.lower() == 'ax12':
            self._result.success = self.sendToAx12(actuator.id, goal.order, param, timeout)
            self._action_server.set_succeeded(self._result)
            return

    def _callback_move_response(self, msg):
        with self._lock:
            if msg.order_nb in self._call_stack:
                event = self._call_stack[msg.order_nb]
                self._call_stack[msg.order_nb] = msg.success
                event.set()
            else:
                rospy.logwarn('Unknow id received : {}'.format(msg.order_nb))

    def sendToArduino(self, ard_id, ard_type, order, param, timeout):
        msg = drivers_ard_others.msg.Move()
        msg.id = ard_id
        msg.type = {
                'digital': msg.TYPE_DIGITAL,
                'pwm': msg.TYPE_PWM,
                'servo': msg.TYPE_SERVO
            }[ard_type]
        msg.dest_value = param
        # TODO do not use event because it is blocking !
        event = threading.Event()
        event.clear()
        msg.order_nb = self.generateId(event)
        self._pub_ard_move.publish(msg)
        ts = current_milli_time()
        event.wait(timeout / 1000.0)
        ts = current_milli_time() - ts
        if ts >= timeout:
            rospy.loginfo('Timeout reached')
        success = False
        if type(self._call_stack[msg.order_nb]) == bool:
            success = self._call_stack[msg.order_nb]
        with self._lock:
            del self._call_stack[msg.order_nb]
        return success

    def sendToAx12(self, id, order, param, timeout):
        goal = drivers_ax12.msg.Ax12CommandGoal()
        goal.motor_id = int(id)
        if order.lower() == "joint":
            goal.mode = drivers_ax12.msg.Ax12CommandGoal.JOINT
            goal.speed = 0
            goal.position = int(param)
        elif order.lower() == "wheel":
            goal.mode = drivers_ax12.msg.Ax12CommandGoal.WHEEL
            goal.speed = int(param)
        else:
            rospy.logerr("Bad order: {}, expected joint or wheel".format(order))
            return False
        self._act_cli_ax12.send_goal(goal)
        # TODO it in an asynchronous way because it's blocking !
        if self._act_cli_ax12.wait_for_result(rospy.Duration(int(timeout))):
            success = self._act_cli_ax12.get_result().success
        else:
            rospy.loginfo('Timeout reached')
            success = False
        return success

    def generateId(self, event):
        with self._lock:
            ard_id = random.randint(1, 10000)
            while ard_id in self._call_stack:
                ard_id = random.randint(1, 10000)
            self._call_stack[ard_id] = event
        return ard_id

