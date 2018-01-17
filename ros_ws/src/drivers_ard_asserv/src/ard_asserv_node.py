#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose2D
import actionlib
from drivers_ard_asserv.srv import *
from drivers_ard_asserv.msg import *
import check_arduino
import asserv

__author__ = "Thomas Fuhrmann"
__date__ = 21/10/2017

NODE_NAME = "ard_asserv"


class Asserv:
    """
    The Asserv class manages the driver node for communication with asserv (real using the Arduino or in simulation).
    As this node is used as an interface, it gets orders to send on ROS services. The state of the robot is published on ROS topics.
    This node handles an action (see actionlib) for the Goto movement.
    The selection of asserv type (real or simu) is made using serial port data. If the Arduino asserv is found, the real asserv is used.
    If no Arduino asserv is found, the node will launch a simulated asserv.
    """
    def __init__(self):
        rospy.logdebug("[ASSERV] Starting asserv_node.")
        # This dictionary stores the goals received by the DoGoto action and which are currently in processing
        self._goals_dictionary = {}
        # This dictionary stores the goals received by the Goto service and which are currently in processing
        self._goto_srv_dictionary = {}
        # Unique ID given to the received goal to manage it
        self._goal_id_counter = 0
        # Instance of the asserv object (simu or real)
        self._asserv_instance = None
        # Init ROS stuff
        rospy.init_node(NODE_NAME, anonymous=False, log_level=rospy.INFO)
        self._pub_robot_pose = rospy.Publisher("/drivers/" + NODE_NAME + "/pose2d", Pose2D, queue_size=5)
        self._pub_robot_speed = rospy.Publisher("/drivers/" + NODE_NAME + "/speed", RobotSpeed, queue_size=5)
        self._srv_goto = rospy.Service("/drivers/" + NODE_NAME + "/goto", Goto, self._callback_goto)
        self._srv_pwm = rospy.Service("/drivers/" + NODE_NAME + "/pwm", Pwm, self._callback_pwm)
        self._srv_speed = rospy.Service("/drivers/" + NODE_NAME + "/speed", Speed, self._callback_speed)
        self._srv_set_pos = rospy.Service("/drivers/" + NODE_NAME + "/set_pos", SetPos, self._callback_set_pos)
        self._srv_emergency_stop = rospy.Service("/drivers/" + NODE_NAME + "/emergency_stop", EmergencyStop, self._callback_emergency_stop)
        self._srv_params = rospy.Service("/drivers/" + NODE_NAME + "/parameters", Parameters, self._callback_asserv_param)
        self._srv_management = rospy.Service("/drivers/" + NODE_NAME + "/management", Management, self._callback_management)
        self._act_goto = actionlib.ActionServer("/drivers/" + NODE_NAME + "/goto_action", DoGotoAction, self._callback_action_goto, auto_start=False)
        self._act_goto.start()
        arduino_port = check_arduino.get_arduino_port("asserv")
        if arduino_port == "":
            rospy.loginfo("[ASSERV] Creation of the simu asserv.")
            self._asserv_instance = asserv.AsservSimu(self)
        else:
            rospy.loginfo("[ASSERV] Creation of the real asserv.")
            self._asserv_instance = asserv.AsservReal(self, arduino_port)
        self._asserv_instance.start()

    # goal_id generated by node, reached to know if the gial as been reached
    def goal_reached(self, goal_id, reached):
        result = DoGotoResult(reached)
        if goal_id in self._goals_dictionary:
            self._goals_dictionary[goal_id].set_succeeded(result)
            del self._goals_dictionary[goal_id]
        elif goal_id in self._goto_srv_dictionary:
            del self._goto_srv_dictionary[goal_id]

    # robot_pose is a Pose2d structure
    def send_robot_position(self, robot_pose):
        self._pub_robot_pose.publish(robot_pose)

    # robot_speed is a RobotSpeed structure
    def send_robot_speed(self, robot_speed):
        self._pub_robot_speed.publish(robot_speed)

    def _callback_goto(self, request):
        """
        Callback of the Goto service.
        @param request: Service request
        @type request:  GotoRequest
        @return:        True if request has been processed, false otherwise
        @rtype:         GotoResponse
        """
        rospy.logdebug("[ASSERV] Received a request (goto service).")
        # TODO manage the direction
        response = self._process_goto_order(self._goal_id_counter, request.mode, request.position.x, request.position.y, request.position.theta, 1)
        if response:
            self._goto_srv_dictionary[self._goal_id_counter] = ""
            self._goal_id_counter += 1
        else:
            rospy.logerr("[ASSERV] Service GOTO has failed... Mode probably does not exist.")
        return GotoResponse(response)

    def _callback_set_pos(self, request):
        """
        Callback of the SetPos service.
        @param request: Service request
        @type request:  SetPosRequest
        @return:        True if request has been processed, false otherwise
        @rtype:         SetPosResponse
        """
        rospy.logdebug("[ASSERV] Received a request (set_pos service).")
        ret_value = self._asserv_instance.set_pos(request.position.x, request.position.y, request.position.theta)
        return SetPosResponse(ret_value)

    def _callback_pwm(self, request):
        """
        Callback of the Pwm service.
        @param request: Service request
        @type request:  PwmRequest
        @return:        True if request has been processed, false otherwise
        @rtype:         PwmResponse
        """
        rospy.logdebug("[ASSERV] Received a request (pwm service).")
        ret_value = self._asserv_instance.pwm(request.left, request.right, request.duration)
        return PwmResponse(ret_value)

    def _callback_speed(self, request):
        """
        Callback of the Speed service.
        @param request: Service request
        @type request:  SpeedRequest
        @return:        True if request has been processed, false otherwise
        @rtype:         SpeedResponse
        """
        rospy.logdebug("[ASSERV] Received a request (speed service).")
        ret_value = self._asserv_instance.speed(request.linear, request.angular, request.duration)
        return SpeedResponse(ret_value)

    def _callback_emergency_stop(self, request):
        """
        Callback of the EmergencyStop service.
        @param request: Service request
        @type request:  EmergencyStopRequest
        @return:        True if request has been processed, false otherwise
        @rtype:         EmergencyStopResponse
        """
        rospy.logdebug("[ASSERV] Received a request (emergency_stop service).")
        ret_value = self._asserv_instance.set_emergency_stop(request.enable)
        return EmergencyStopResponse(ret_value)

    def _callback_asserv_param(self, request):
        """
        Callback of the Parameters service.
        @param request: Service request
        @type request:  ParametersRequest
        @return:        True if request has been processed, false otherwise
        @rtype:         ParametersResponse
        """
        rospy.logdebug("[ASSERV] Received a request (parameters service).")
        response = True
        if request.mode == request.SPDMAX:
            response = self._asserv_instance.set_max_speed(request.spd, request.spd_ratio)
        elif request.mode == request.ACCMAX:
            response = self._asserv_instance.set_max_accel(request.acc)
        elif request.mode == request.PIDRIGHT:
            # TODO manage left and right
            response = self._asserv_instance.set_pid(request.p, request.i, request.d)
        elif request.mode == request.PIDLEFT:
            # TODO manage left and right
            response = self._asserv_instance.set_pid(request.p, request.i, request.d)
        elif request.mode == request.PIDALL:
            response = self._asserv_instance.set_pid(request.p, request.i, request.d)
        else:
            response = False
            rospy.logerr("[ASSERV] Parameter mode %d does not exists...", request.mode)
        return ParametersResponse(response)

    def _callback_management(self, request):
        """
        Callback of the Management service.
        @param request: Service request
        @type request:  ManagementRequest
        @return:        True if request has been processed, false otherwise
        @rtype:         ManagementResponse
        """
        rospy.logdebug("[ASSERV] Received a request (management service).")
        response = True
        if request.mode == request.KILLG:
            response = self._asserv_instance.kill_goal()
        elif request.mode == request.CLEANG:
            response = self._asserv_instance.clean_goals()
            # Delete all internal goals
            for goal in self._goals_dictionary.values():
                goal.set_canceled()
            self._goals_dictionary.clear()
        elif request.mode == request.PAUSE:
            response = self._asserv_instance.pause(True)
        elif request.mode == request.RESUME:
            response = self._asserv_instance.pause(False)
        elif request.mode == request.RESET_ID:
            response = self._asserv_instance.reset_id()
        else:
            response = False
            rospy.logerr("[ASSERV] Management mode %d does not exists...", request.mode)
        return ManagementResponse(response)

    def _callback_action_goto(self, goal_handled):
        """
        Callback of the DoGoto action.
        @param goal_handled:    Goal handler corresponding to the received action
        @type goal_handled:     ServerGoalHandle
        """
        rospy.logdebug("[ASSERV] Received a request (dogoto action).")
        #TODO manage direction
        if self._process_goto_order(self._goal_id_counter, goal_handled.get_goal().mode, goal_handled.get_goal().position.x, goal_handled.get_goal().position.y, goal_handled.get_goal().position.theta, 1):
            goal_handled.set_accepted()
            self._goals_dictionary[self._goal_id_counter] = goal_handled
            self._goal_id_counter += 1
        else:
            rospy.logerr("[ASSERV] Action GOTO has failed... Mode probably does not exist.")

    def _process_goto_order(self, goal_id, mode, x, y, a, direction):
        """
        Processes the goto order, coming from service or action.
        @param mode:    Mode of the Goto order (see Goto.srv or DoGoto.action files)
        @type mode:     string
        @param x:       X coordinate (in meters)
        @type x:        float64
        @param y:       Y coordinate (in meters)
        @type y:        float64
        @param a:       Angle (in radians)
        @type a:        float64
        @return:        True if order sent, false otherwise
        @rtype:         bool
        """
        to_return = True
        if mode == GotoRequest.GOTO:
            to_return = self._asserv_instance.goto(goal_id, x, y, direction)
        elif mode == GotoRequest.GOTOA:
            to_return = self._asserv_instance.gotoa(goal_id, x, y, a, direction)
        elif mode == GotoRequest.ROT:
            to_return = self._asserv_instance.rot(goal_id, a, False)
        elif mode == GotoRequest.ROTNOMODULO:
            to_return = self._asserv_instance.rot(goal_id, a, True)
        else:
            to_return = False
            rospy.logerr("[ASSERV] GOTO mode %d does not exists...", mode)
        return to_return


if __name__ == "__main__":
    Asserv()
