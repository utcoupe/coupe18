#!/usr/bin/env python

import rospy
import threading
from functools import partial
from actuators_abstract import *
import actuators_parser
from movement_actuators.msg import DispatchAction, DispatchResult
from drivers_ax12.msg import Ax12CommandGoal, Ax12CommandAction
from actionlib_msgs.msg import GoalStatus
import drivers_ard_others.msg
import drivers_ax12.msg

__author__ = "Thomas Fuhrmann"
__date__ = 9/04/2018


class ActuatorsDispatch(ActuatorsAbstract):
    def __init__(self):
        ActuatorsAbstract.__init__(self, "dispatch", DispatchAction)
        self._lock = threading.RLock()
        # order_id_counter, goal_id
        self._active_goals_arduino = {}
        # ax_client_goal, goal_id
        self._active_goals_ax12 = {}
        self._active_goal_timers = {}
        # Those counters enables to keep a track of orders sent
        self._order_id_counter = 0
        self._act_parser = actuators_parser.ActuatorsParser()
        self._pub_ard_move = rospy.Publisher('/drivers/ard_others/move', drivers_ard_others.msg.Move, queue_size=3)
        self._sub_ard_response = rospy.Subscriber('/drivers/ard_others/move_response', drivers_ard_others.msg.MoveResponse, self._callback_move_response)
        self._act_cli_ax12 = actionlib.ActionClient('/drivers/ax12', Ax12CommandAction)
        self._act_cli_ax12.wait_for_server(rospy.Duration(10))

    def _process_action(self, goal, goal_id):
        to_return = False
        actuator_properties = self._act_parser.get_actuator_properties(goal.name, goal.id)
        if actuator_properties is None:
            return False
        param = goal.param
        # TODO add more checks !
        if param == "":
            if goal.preset != "":
                param = actuator_properties.preset[goal.preset]
        if actuator_properties.family.lower() == 'arduino':
            result = self._send_to_arduino(actuator_properties.id, actuator_properties.type, param)
            if result >= 0:
                with self._lock:
                    self._active_goals_arduino[result] = goal_id
                to_return = True
        elif actuator_properties.family.lower() == 'ax12':
            goal_handle = self._send_to_ax12(actuator_properties.id, goal.order, param)
            if goal_handle is not None:
                with self._lock:
                    self._active_goals_ax12[goal_handle] = goal_id
                to_return = True
            else:
                to_return = False
        if to_return:
            if goal.timeout > 0:
                self._active_goal_timers[goal_id] = rospy.Timer(period=rospy.Duration(goal.timeout / 1000),
                                                                callback=partial(self._timer_callback_timeout, goal_id=goal_id),
                                                                oneshot=True)
        return to_return

    def _timer_callback_timeout(self, event, goal_id):
        rospy.logerr('Timeout triggered for goal %s, cancelling' % goal_id.id)
        for goal in self._active_goals_ax12.iterkeys():
            if self._active_goals_ax12[goal] == goal_id:
                self._action_reached(self._active_goals_ax12[goal], False, DispatchResult(False))
                del self._active_goals_ax12[goal]
        for goal in self._active_goals_arduino.iterkeys():
            if self._active_goals_arduino[goal] == goal_id:
                self._action_reached(self._active_goals_arduino[goal], False, DispatchResult(False))
                del self._active_goals_arduino[goal]

    def _callback_move_response(self, msg):
        goal_found = False
        with self._lock:
            # TODO improve
            for key in self._active_goals_arduino.iterkeys():
                if msg.order_nb == key:
                    goal_found = True
                    self._action_reached(self._active_goals_arduino[key], msg.success, DispatchResult(msg.success))
                    del self._active_goals_arduino[key]
                    if key in self._active_goal_timers:
                        self._active_goal_timers[key].shutdown()
                        del self._active_goal_timers[key]
        if not goal_found:
            rospy.logwarn('Unknow id received : {}'.format(msg.order_nb))

    def _callback_ax12_client(self, goal_handle):
        received_goal_id = goal_handle.comm_state_machine.action_goal.goal_id.id
        for goal in self._active_goals_ax12:
            if goal.comm_state_machine.action_goal.goal_id.id == received_goal_id:
                goal_status = goal_handle.get_goal_status()

                if goal_status == GoalStatus.SUCCEEDED:
                    success = True
                elif goal_status in [GoalStatus.LOST,
                              GoalStatus.RECALLED,
                              GoalStatus.REJECTED,
                              GoalStatus.ABORTED,
                              GoalStatus.PREEMPTED]:
                    success = False
                else:
                    return

                self._action_reached(self._active_goals_ax12[goal], success, DispatchResult(success))
                del self._active_goals_ax12[goal]
                return

    def _send_to_arduino(self, ard_id, ard_type, param):
        msg = drivers_ard_others.msg.Move()
        msg.id = ard_id
        msg.type = {
                'digital': msg.TYPE_DIGITAL,
                'pwm': msg.TYPE_PWM,
                'servo': msg.TYPE_SERVO
            }[ard_type]
        msg.dest_value = param
        msg.order_nb = self._get_order_id()
        self._pub_ard_move.publish(msg)
        return msg.order_nb

    def _send_to_ax12(self, motor_id, order, param):
        try:
            goal_position = int(param)
        except ValueError as ex:
            rospy.logerr("Try to dispatch an ax12 order with invalid position... Abort it.")
            return None
        goal = drivers_ax12.msg.Ax12CommandGoal()
        goal.motor_id = int(motor_id)
        if order.lower() == "joint":
            goal.mode = drivers_ax12.msg.Ax12CommandGoal.JOINT
            goal.speed = 0
            goal.position = goal_position
        elif order.lower() == "wheel":
            goal.mode = drivers_ax12.msg.Ax12CommandGoal.WHEEL
            goal.speed = goal_position
        else:
            rospy.logerr("Bad order: {}, expected joint or wheel".format(order))
            return None
        return self._act_cli_ax12.send_goal(goal, transition_cb=self._callback_ax12_client)

    def _get_order_id(self):
        with self._lock:
            current_id = self._order_id_counter
            self._order_id_counter += 1
        return current_id
