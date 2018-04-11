#!/usr/bin/env python

import rospy
import threading
from actuators_abstract import *
import actuators_parser
from movement_actuators.msg import DispatchAction, DispatchResult
import drivers_ard_others.msg
import drivers_ax12.msg

__author__ = "Thomas Fuhrmann"
__date__ = 9/04/2018


class ActuatorsDispatch(ActuatorsAbstract):
    def __init__(self):
        ActuatorsAbstract.__init__(self, "dispatch", DispatchAction)
        self._lock = threading.RLock()
        # TODO add timeout management
        # dispatch_id (order_nb for arduino), goal_id
        self._active_goals = {}
        self._ard_order_id = 0
        self._act_parser = actuators_parser.ActuatorsParser()
        self._pub_ard_move = rospy.Publisher('/drivers/ard_others/move', drivers_ard_others.msg.Move, queue_size=3)
        self._sub_ard_response = rospy.Subscriber('/drivers/ard_others/move_response', drivers_ard_others.msg.MoveResponse, self._callback_move_response)
        # TODO implement an ActionClient instead of SimpleActionClient
        # self._act_cli_ax12 = actionlib.SimpleActionClient('/drivers/ax12', drivers_ax12.msg.Ax12CommandAction)

    def _process_action(self, goal, goal_id):
        result = False
        actuator_properties = self._act_parser.get_actuator_properties(goal.name, goal.id)
        if actuator_properties is None:
            return False
        if actuator_properties.family.lower() == 'arduino':
            param = goal.param
            # TODO add more checks !
            if param == "":
                param = actuator_properties.preset[goal.preset]
            result = self.sendToArduino(actuator_properties.id, actuator_properties.type, param)
        if result:
            with self._lock:
                # TODO it better + checks
                self._active_goals[self._ard_order_id - 1] = goal_id
        # TODO implement ax12 management
        # elif actuator_properties.family.lower() == 'ax12':
        #     result = self.sendToAx12(actuator.id, goal.order, param, timeout)
        return result

    def _callback_move_response(self, msg):
        goal_found = False
        with self._lock:
            # TODO improve
            for key in self._active_goals.iterkeys():
                if msg.order_nb == key:
                    goal_found = True
                    self._action_reached(self._active_goals[key], msg.success, DispatchResult(msg.success))
        if not goal_found:
            rospy.logwarn('Unknow id received : {}'.format(msg.order_nb))

    def sendToArduino(self, ard_id, ard_type, param):
        msg = drivers_ard_others.msg.Move()
        msg.id = ard_id
        msg.type = {
                'digital': msg.TYPE_DIGITAL,
                'pwm': msg.TYPE_PWM,
                'servo': msg.TYPE_SERVO
            }[ard_type]
        msg.dest_value = param
        msg.order_nb = self._ard_order_id
        self._ard_order_id += 1
        self._pub_ard_move.publish(msg)
        return True

    # def sendToAx12(self, id, order, param, timeout):
    #     goal = drivers_ax12.msg.Ax12CommandGoal()
    #     goal.motor_id = int(id)
    #     if order.lower() == "joint":
    #         goal.mode = drivers_ax12.msg.Ax12CommandGoal.JOINT
    #         goal.speed = 0
    #         goal.position = int(param)
    #     elif order.lower() == "wheel":
    #         goal.mode = drivers_ax12.msg.Ax12CommandGoal.WHEEL
    #         goal.speed = int(param)
    #     else:
    #         rospy.logerr("Bad order: {}, expected joint or wheel".format(order))
    #         return False
    #     self._act_cli_ax12.send_goal(goal)
    #     # TODO it in an asynchronous way because it's blocking !
    #     if self._act_cli_ax12.wait_for_result(rospy.Duration(int(timeout))):
    #         success = self._act_cli_ax12.get_result().success
    #     else:
    #         rospy.loginfo('Timeout reached')
    #         success = False
    #     return success
