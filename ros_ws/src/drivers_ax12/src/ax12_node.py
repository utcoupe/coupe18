#!/usr/bin/env python

import rospy
import actionlib
from drivers_ax12.msg import *
from drivers_port_finder.srv import GetPort
import subprocess

class Ax12:

    def __init__(self):
        self.BAUD_RATE = 1000000
        self.SCAN_RANGE = 10
        self.PROFILE_VELOCITY = 200
        self.PROFILE_ACCELERATION = 50

        self.GET_PORT_SERVICE_NAME = "/drivers/port_finder/get_port"

        rospy.init_node('ax12', anonymous=False)

        #self.current_goals = {}
        #self.next_goal_id = 0

        #self._act_goto = actionlib.ActionServer("ax12/command_action", AngleCommandAction,
        #                                        self.action_callback, auto_start=False)
        #self._act_goto.start()

        self.port = self.fetch_port()
        self.init_params()
        self.run_position_control_node()
        rospy.loginfo("Position control node launched, ax12_node shutting down...")


    def fetch_port(self):
        try:
            rospy.wait_for_service(self.GET_PORT_SERVICE_NAME, 10)
            port = rospy.ServiceProxy(self.GET_PORT_SERVICE_NAME, GetPort)("usb2ax").port
            if not port:
                rospy.logerr("Error when fetching the port for usb2ax")
            return port
        except rospy.ROSException:
            rospy.logerr("Port_finder service does not exist, can't fetch the ax12 port...")

    def init_params(self):
        rospy.set_param('device_name', self.port)
        rospy.set_param('baud_rate', self.BAUD_RATE)
        rospy.set_param('scan_range', self.SCAN_RANGE)
        rospy.set_param('profile_velocity', self.PROFILE_VELOCITY)
        rospy.set_param('profile_acceleration', self.PROFILE_ACCELERATION)

    def run_position_control_node(self):
        self.node_subprocess = subprocess.Popen(['rosrun', 'dynamixel_workbench_controllers', 'position_control',
                                                 "__ns:=" + rospy.get_namespace()])


    def action_callback(self, goal_handled):
        goal = goal_handled.get_goal()
        self.add_goal(goal_handled)


    def add_goal(self, goal_handled):
        self.current_goals[self.next_goal_id] = goal_handled
        self.next_goal_id += 1
        goal_handled.set_accepted()

    def remove_goal(self, goal_id):
        self.current_goals.pop(goal_id)


if __name__ == "__main__":
    Ax12()
