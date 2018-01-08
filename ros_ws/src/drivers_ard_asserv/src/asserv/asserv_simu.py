#!/usr/bin/env python

import math
import rospy
from asserv_abstract import *
from geometry_msgs.msg import Pose2D
from drivers_ard_asserv.msg import RobotSpeed

__author__ = "Thomas Fuhrmann"
__date__ = 16/12/2017

# TODO adapt to have realistic behaviour of the robot
SEND_POSE_RATE = 0.1  # in ms
SEND_SPEED_RATE = 0.1  # in ms
ASSERV_RATE = 0.05  # in ms
ASSERV_ERROR_POSITION = 0.02  # in meters


class AsservSimu(AsservAbstract):
    def __init__(self, asserv_node):
        AsservAbstract.__init__(self, asserv_node)
        rospy.loginfo("AsservSimu")
        # Asserv management stuff
        # The pose is in meters and rad
        self._current_pose = Pose2D(0, 0, 0)
        self._current_linear_speed = 0
        self._current_angular_speed = 0
        self._currently_moving = False
        self._first_trajectory_update = True
        self._goal_distance = 0
        # TODO rename
        self._goal_angle = 0
        self._current_goal_position = Pose2D(0, 0, 0)
        # Parameters
        # The acceleration is in m/s^2
        self._max_acceleration = 0.2
        # The speed is in m/s
        self._max_linear_speed = 0.5
        # The angular speed is in rad/s
        self._max_angular_speed = 0.01
        self._emergency_stop_flag = False
        # ROS stuff
        self._tmr_pose_send = rospy.Timer(rospy.Duration(SEND_POSE_RATE), self._callback_timer_pose_send)
        self._tmr_speed_send = rospy.Timer(rospy.Duration(SEND_SPEED_RATE), self._callback_timer_speed_send)
        self._tmr_asserv_computation = rospy.Timer(rospy.Duration(ASSERV_RATE), self._callback_timer_asserv_computation)

    def start(self):
        rospy.logdebug("[ASSERV] Node has correctly started in simulation mode.")
        while not rospy.is_shutdown():
            rospy.sleep(0.01)

    def goto(self, goal_id, x, y, direction):
        self._current_goal_position = Pose2D(x, y, 0)
        rospy.loginfo("[ASSERV] Accepting goal (x = " + str(self._current_goal_position.x) + ", y = " + str(self._current_goal_position.y) + ").")
        self._start_trajectory(x, y)
        return True

    def gotoa(self, goal_id, x, y, a, direction):
        rospy.logerr("AsservAbstract is abstract !")
        return False

    def rot(self, goal_id, a, no_modulo):
        rospy.logerr("AsservAbstract is abstract !")
        return False

    def pwm(self, left, right, duration):
        rospy.logwarn("Pwm is not implemented in simu yet...")
        return False

    def speed(self, linear, angular, duration):
        rospy.logerr("AsservAbstract is abstract !")
        return False

    def set_emergency_stop(self, stop):
        rospy.logerr("AsservAbstract is abstract !")
        return False

    def kill_goal(self):
        rospy.logerr("AsservAbstract is abstract !")
        return False

    def clean_goals(self):
        rospy.logerr("AsservAbstract is abstract !")
        return False

    def pause(self, pause):
        rospy.logwarn("Pause is not implemented in simu yet...")
        return False

    def reset_id(self):
        return True

    def set_max_speed(self, speed, speed_ratio):
        self._max_linear_speed = speed
        self._max_angular_speed = speed * speed_ratio
        return True

    def set_max_accel(self, accel):
        self._max_acceleration = accel
        return True

    def set_pid(self, p, i, d):
        return True

    def set_pos(self, x, y, a):
        if self._currently_moving:
            # TODO set the pose or return false ?
            rospy.logwarn("Setting pose wile moving is not a good idea...")
        else:
            self._current_pose = Pose2D(x, y, a)
        return True

    def _callback_timer_asserv_computation(self, event):
        if self._currently_moving:
            # Check if the robot is arrived
            if ((self._current_pose.x > self._current_goal_position.x - ASSERV_ERROR_POSITION) and
                    (self._current_pose.x < self._current_goal_position.x + ASSERV_ERROR_POSITION) and
                    (self._current_pose.y > self._current_goal_position.y - ASSERV_ERROR_POSITION) and
                    (self._current_pose.y < self._current_goal_position.y + ASSERV_ERROR_POSITION)):
                rospy.loginfo("[ASSERV] Goal position has been reached !")
                self._currently_moving = False
                self._current_linear_speed = 0
            else:
                # TODO deceleration
                self._current_linear_speed += self._max_acceleration * ASSERV_RATE
                if self._current_linear_speed > self._max_linear_speed:
                    self._current_linear_speed = self._max_linear_speed
                current_x = self._current_pose.x + self._current_linear_speed * ASSERV_RATE * math.cos(self._goal_angle)
                current_y = self._current_pose.y + self._current_linear_speed * ASSERV_RATE * math.sin(self._goal_angle)
                self._current_pose = Pose2D(current_x, current_y, self._current_pose.theta)

    def _callback_timer_pose_send(self, event):
        self._node.send_robot_position(self._current_pose)

    def _callback_timer_speed_send(self, event):
        # TODO
        self._node.send_robot_speed(RobotSpeed(0, 0, self._current_linear_speed, 0, 0))

    def _start_trajectory(self, x, y, a = 0, direction = "forward"):
        # TODO goal management
        # TODO end_position management
        self._currently_moving = True
        self._goal_distance = ((x - self._current_pose.x) ** 2 + (y - self._current_pose.y) ** 2) ** 0.5
        # TODO this is not good, wrong computation !!!
        # self._goal_angle = math.acos((self._current_pose.x * x + self._current_pose.y * y) / self._goal_distance)
        self._goal_angle = math.acos(x / self._goal_distance)
        rospy.loginfo("distance : " + str(self._goal_distance) + " angle : " + str(self._goal_angle))
