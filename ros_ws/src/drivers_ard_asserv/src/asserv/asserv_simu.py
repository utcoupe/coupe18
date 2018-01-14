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
# To be more accurate on position error, the speed has to be lower otherwise the goal reached check will fail
ASSERV_ERROR_POSITION = 0.005  # in meters
ASSERV_MINIMAL_SPEED = 0.05 # in m/s

# TODO angle goal management
# TODO actionlib goals management (goal_id) --> put it in the abstract class ?


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
        self._emergency_stop = False
        self._current_goal = Pose2D(0, 0, 0)
        # Distance between the robot and the goal when the current goal is set
        self._current_goal_initial_distance = 0
        # Angle between the robot and the goal when the current goal is set
        self._current_goal_initial_angle = 0
        # List of Pose2d corresponding to the goals
        self._goals_list = []
        # Parameters
        # The acceleration is in m/s^2
        self._max_acceleration = 0.1
        # The speed is in m/s
        self._max_linear_speed = 0.5
        # The angular speed is in rad/s
        self._max_angular_speed = 0.01
        # ROS stuff
        self._tmr_pose_send = rospy.Timer(rospy.Duration(SEND_POSE_RATE), self._callback_timer_pose_send)
        self._tmr_speed_send = rospy.Timer(rospy.Duration(SEND_SPEED_RATE), self._callback_timer_speed_send)
        self._tmr_asserv_computation = rospy.Timer(rospy.Duration(ASSERV_RATE), self._callback_timer_asserv_computation)

    def start(self):
        rospy.logdebug("[ASSERV] Node has correctly started in simulation mode.")
        while not rospy.is_shutdown():
            rospy.sleep(0.01)

    def goto(self, goal_id, x, y, direction):
        rospy.loginfo("[ASSERV] Accepting goal (x = " + str(x) + ", y = " + str(y) + ").")
        self._start_trajectory(x, y)
        return True

    def gotoa(self, goal_id, x, y, a, direction):
        rospy.loginfo("[ASSERV] Accepting goal (x = " + str(x) + ", y = " + str(y) + "), angle has not been implemented yet...")
        self._start_trajectory(x, y)
        return True

    def rot(self, goal_id, a, no_modulo):
        rospy.logerr("This function has not been implemented yet...")
        return False

    def pwm(self, left, right, duration):
        rospy.logwarn("Pwm is not implemented in simu yet...")
        return False

    def speed(self, linear, angular, duration):
        rospy.logerr("This function has not been implemented yet...")
        return False

    def set_emergency_stop(self, stop):
        self._emergency_stop = stop
        if stop:
            self._current_linear_speed = 0
        return True

    def kill_goal(self):
        rospy.logerr("This function has not been implemented yet...")
        return False

    def clean_goals(self):
        self._goals_list = []
        return True

    def pause(self, pause):
        rospy.logerr("This function has not been implemented yet...")
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
        return_value = True
        if self._currently_moving:
            rospy.logwarn("Setting pose wile moving is not a good idea...")
            return_value = False
        else:
            self._current_pose = Pose2D(x, y, a)
        return return_value

    def _callback_timer_asserv_computation(self, event):
        # First check if a goal has to be got from the list
        if self._current_goal == Pose2D(0, 0, 0) and len(self._goals_list) > 0:
            self._current_goal = self._goals_list.pop()
            rospy.loginfo("[ASSERV] Starting a new goal : x = " + str(self._current_goal.x) + " y = " + str(self._current_goal.y) + " a = " + str(self._current_goal.theta))
            self._current_goal_initial_distance = ((self._current_goal.x - self._current_pose.x) ** 2 + (self._current_goal.y - self._current_pose.y) ** 2) ** 0.5
            self._current_goal_initial_angle = math.atan2(self._current_goal.y - self._current_pose.y, self._current_goal.x - self._current_pose.x)
            self._currently_moving = True
        if self._currently_moving and not self._emergency_stop:
            # Check if the robot is arrived
            if ((self._current_pose.x > self._current_goal.x - ASSERV_ERROR_POSITION) and
                    (self._current_pose.x < self._current_goal.x + ASSERV_ERROR_POSITION) and
                    (self._current_pose.y > self._current_goal.y - ASSERV_ERROR_POSITION) and
                    (self._current_pose.y < self._current_goal.y + ASSERV_ERROR_POSITION)):
                rospy.loginfo("[ASSERV] Goal position has been reached !")
                self._current_goal = Pose2D(0, 0, 0)
                self._currently_moving = False
                self._current_linear_speed = 0
            else:
                current_goal_distance = ((self._current_goal.x - self._current_pose.x) ** 2 + (self._current_goal.y - self._current_pose.y) ** 2) ** 0.5
                if current_goal_distance > self._current_goal_initial_distance / 2:
                    self._current_linear_speed += self._max_acceleration * ASSERV_RATE
                else:
                    self._current_linear_speed -= self._max_acceleration * ASSERV_RATE
                    if self._current_linear_speed < ASSERV_MINIMAL_SPEED:
                        self._current_linear_speed = ASSERV_MINIMAL_SPEED
                if self._current_linear_speed > self._max_linear_speed:
                    self._current_linear_speed = self._max_linear_speed
                current_x = self._current_pose.x + self._current_linear_speed * ASSERV_RATE * math.cos(self._current_goal_initial_angle)
                current_y = self._current_pose.y + self._current_linear_speed * ASSERV_RATE * math.sin(self._current_goal_initial_angle)
                # TODO modify current angle
                self._current_pose = Pose2D(current_x, current_y, self._current_pose.theta)

    def _callback_timer_pose_send(self, event):
        self._node.send_robot_position(self._current_pose)

    def _callback_timer_speed_send(self, event):
        self._node.send_robot_speed(RobotSpeed(0, 0, self._current_linear_speed, 0, 0))

    def _start_trajectory(self, x, y, a=0, direction="forward"):
        # TODO use angle
        # TODO use direction ?
        self._goals_list.append(Pose2D(x, y, a))
