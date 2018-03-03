#!/usr/bin/env python

import Queue
import serial
import threading
import os
import rospy
from geometry_msgs.msg import Pose2D
from drivers_ard_asserv.msg import RobotSpeed
from asserv_abstract import *
import protocol_parser

__author__ = "Thomas Fuhrmann"
__date__ = 16/12/2017

ASSERV_ERROR_POSITION = 0.01  # in meters
ASSERV_ERROR_ANGLE = 0.05  # in radians
POSITION_REACHED_CHECK_DELAY = 1  # in seconds


class AsservReal(AsservAbstract):
    def __init__(self, asserv_node, port):
        AsservAbstract.__init__(self, asserv_node)
        rospy.loginfo("AsservReal")
        # Dictionary containing the list of orders which are interpreted by the Arduino (do not modify this dictionary !)
        self._orders_dictionary = protocol_parser.protocol_parse(os.environ['UTCOUPE_WORKSPACE'] + "/arduino/common/asserv/protocol.h")
        # Queue to store the received information from the Arduino
        self._reception_queue = Queue.Queue()
        # A queue is used to send data to prevent to send data too fast, which will result to concatenate two sending and making the Arduino crash
        self._sending_queue = Queue.Queue()
        # The order_id is sent to the asserv to identify the orders, it must be unique
        self._order_id = 0
        # This dictionary stores the order_id with th goal_id to retrieve which goal has been completed
        self._orders_id_dictionary = {}
        # This list stores the last received ack ID from Arduino (a list to avoid zapping an ack). This list is used by the timer callback which check that position has been reached
        self._last_received_id_dictionary = {}
        # Store the current position of robot, this one is the raw value returned by the asserv
        self._robot_raw_position = Pose2D(0, 0, 0)
        # Timer for the send of serial data to avoid that sending are buffered in the internal OS buffer (make the arduino crash)
        self._tmr_serial_send = rospy.Timer(rospy.Duration(0.1), self._callback_timer_serial_send)
        # Init the serial communication
        # Flag to tell that the connected Arduino is started : we can send it orders
        self._arduino_started_flag = False
        # Serial object
        self._serial_com = None
        # Thread dedicated to the reception from the serial line
        self._serial_receiver_thread = None
        self._start_serial_com_line(port)

    def start(self):
        """
        Starts the processing : feeding the reception queue.
        """
        rospy.logdebug("[ASSERV] Node has correctly started.")
        while not rospy.is_shutdown():
            if not self._reception_queue.empty():
                try:
                    self._process_received_data(self._reception_queue.get())
                    self._reception_queue.task_done()
                except KeyboardInterrupt:
                    break
            rospy.sleep(0.01)
        # Force the arduino to halt
        self._halt()

    def goto(self, goal_id, x, y, direction):
        self._send_serial_data(self._orders_dictionary['GOTO'], [str(int(round(x * 1000))), str(int(round(y * 1000))), str(direction)])
        # TODO make it proper
        self._orders_id_dictionary[self._order_id - 1] = [goal_id, x, y]
        return True

    def gotoa(self, goal_id, x, y, a, direction):
        self._send_serial_data(self._orders_dictionary['GOTOA'], [str(int(round(x * 1000))), str(int(round(y * 1000))), str(int(round(a * 1000))), str(direction)])
        # TODO make it proper
        self._orders_id_dictionary[self._order_id - 1] = [goal_id, x, y, a]
        return True

    def rot(self, goal_id, a, no_modulo):
        if no_modulo:
            self._send_serial_data(self._orders_dictionary['ROTNOMODULO'], [str(int(round(a * 1000)))])
        else:
            self._send_serial_data(self._orders_dictionary['ROT'], [str(int(round(a * 1000)))])
        # TODO make it proper
        self._orders_id_dictionary[self._order_id - 1] = [goal_id, a]
        return True

    def pwm(self, left, right, duration):
        self._send_serial_data(self._orders_dictionary['PWM'], [str(left), str(right), str(duration)])
        return True

    def speed(self, linear, angular, duration):
        self._send_serial_data(self._orders_dictionary['SPD'], [str(linear), str(angular), str(duration)])
        return True

    def set_emergency_stop(self, stop):
        self._send_serial_data(self._orders_dictionary['SETEMERGENCYSTOP'], [str(int(stop))])
        return True

    def kill_goal(self):
        self._send_serial_data(self._orders_dictionary['KILLG'], [])
        return True

    def clean_goals(self):
        self._send_serial_data(self._orders_dictionary['CLEANG'], [])
        return True

    def pause(self, pause):
        if pause:
            self._send_serial_data(self._orders_dictionary['PAUSE'], [])
        else:
            self._send_serial_data(self._orders_dictionary['RESUME'], [])
        return True

    def reset_id(self):
        self._send_serial_data(self._orders_dictionary['RESET_ID'], [])
        return True

    def set_max_speed(self, speed, speed_ratio):
        self._send_serial_data(self._orders_dictionary['SPDMAX'], [str(speed), str(speed_ratio)])
        return True

    def set_max_accel(self, accel):
        self._send_serial_data(self._orders_dictionary['ACCMAX'], [str(accel)])
        return True

    def set_pid(self, p, i, d):
        # TODO manage lef and right
        self._send_serial_data(self._orders_dictionary['PIDALL'], [str(int(round(p * 1000))), str(int(round(i * 1000))), str(int(round(d * 1000)))])
        # rospy.loginfo("Set pid sending : P = {}, I = {}, D = {}.".format(str(int(round(p * 1000))), str(int(round(i * 1000))), str(int(round(d * 1000)))))
        return True

    def set_pos(self, x, y, a):
        self._send_serial_data(self._orders_dictionary['SET_POS'], [str(int(round(x * 1000))), str(int(round(y * 1000))), str(int(round(a * 1000.0)))])
        return True

    def _start_serial_com_line(self, port):
        """
        Initiate the serial communication line.
        @param port:    The port where the Arduino is connected (ex : "/devttyUSB0")
        @type port:     string
        """
        try:
            self._serial_com = serial.Serial(port, 57600, timeout=0.5)
            self._serial_receiver_thread = threading.Thread(target=self._data_receiver)
            self._serial_receiver_thread.start()
            rospy.logdebug("[ASSERV] Serial communication line has started.")
        except serial.SerialException:
            rospy.logerr("[ASSERV] Port : " + port + " is not available, make sure you have plugged the Arduino.")

    def _data_receiver(self):
        """
        Receives data from serial communication line and put it in a queue.
        This function has to be called within a thread.
        """
        while not rospy.is_shutdown():
            try:
                received_data = self._serial_com.readline()
                if received_data != "":
                    self._reception_queue.put(received_data.replace('\r\n', ''))
            except KeyboardInterrupt:
                break
            rospy.sleep(0.01)

    def _process_received_data(self, data):
        """
        This function is in charge of processing all the received data from the Arduino.
        The data received have different types :
            * Identification of the Arduino (ex : "asserv_gr")
            * Status data (robot position, speed...), starting with ~
            * Orders ack, tells that an order has finished
            * Debug string
        @param data:    The data received from Arduino
        @type data:     string
        """
        # At init, start the Arduino
        # TODO split data by _ to check if pr or gr and of asserv or others
        if (not self._arduino_started_flag) and data.find("asserv") != -1:
            rospy.logdebug("[ASSERV] Asserv Arduino identified, start it.")
            self._sending_queue.put(self._orders_dictionary['START'] + ";0;\n")
        # Received status
        elif data.find("~") != -1:
            rospy.logdebug("[ASSERV] Received status data.")
            receied_data_list = data.split(";")
            try:
                robot_position = Pose2D(float(receied_data_list[2]) / 1000.0, float(receied_data_list[3]) / 1000.0, float(receied_data_list[4]) / 1000.0)
                self._robot_raw_position = robot_position
                self._node.send_robot_position(robot_position)
                self._node.send_robot_speed(RobotSpeed(float(receied_data_list[5]), float(receied_data_list[6]), float(receied_data_list[7]) / 1000.0, float(receied_data_list[8]), float(receied_data_list[9])))
            except:
                rospy.logwarn("[ASSERV] Received bad position from the robot, drop it...")
        # Received order ack
        elif data.find(";") >= 1 and data.find(";") <= 3:
            # Special order ack, the first one concern the Arduino activation
            if data.find("0;") == 0:
                rospy.loginfo("[ASSERV] Arduino started")
                self._arduino_started_flag = True
                self._order_id += 1
            else:
                rospy.logdebug("[ASSERV] Received order ack : %s", data)
                ack_data = data.split(";")
                try:
                    ack_id = int(ack_data[0])
                except:
                    ack_id = -1
                # TODO manage status
                if ack_id in self._orders_id_dictionary:
                    rospy.logdebug("[ASSERV] Found key %d in order_id dictionary !", ack_id)
                    self._last_received_id_dictionary[ack_id] = self._orders_id_dictionary[ack_id]
                    del self._orders_id_dictionary[ack_id]
                    self._check_reached_timer = rospy.Timer(rospy.Duration(POSITION_REACHED_CHECK_DELAY), self._callback_timer_check_reached, oneshot=True)
                else:
                    # Do nothing, some IDs are returned but do not correspond to a value in the dictionary.
                    rospy.logdebug("Received ack id ({}) but dropping it.".format(ack_id))
        else:
            rospy.loginfo("%s", data)

    def _send_serial_data(self, order_type, args_list):
        """
        This function sends data to the Arduino, using the specific protocol.
        @param order_type:  Type of the order to send (see protocol.h in arduino/common/asserv folder)
        @type order_type:   string containing an int
        @param args_list:   List of arguments, depending on the order
        @type args_list:    List
        """
        if self._serial_com is not None:
            args_list.insert(0, str(self._order_id))
            self._order_id += 1
            # TODO check if \n is necessary  + '\n'
            self._sending_queue.put(order_type + ";" + ";".join(args_list) + ";\n")
        else:
            rospy.logerr("[ASSERV] Try to send data but serial line is not connected...")

    def _callback_timer_serial_send(self, event):
        if not self._sending_queue.empty():
            data_to_send = self._sending_queue.get()
            rospy.logdebug("Sending data : " + data_to_send)
            self._serial_com.write(data_to_send)
            self._sending_queue.task_done()

    def _check_reached_angle(self, a):
        rospy.loginfo("Check reached angle, own angle = {}, check angle  = {}".format(self._robot_raw_position.theta, a))
        return (self._robot_raw_position.theta > a - ASSERV_ERROR_ANGLE) and (self._robot_raw_position.theta < a + ASSERV_ERROR_ANGLE)

    def _check_reached_position(self, x, y):
        rospy.loginfo("Check reached position, own pos = {}, {}, check pos  = {}, {}".format(self._robot_raw_position.x, self._robot_raw_position.y, x, y))
        return ((self._robot_raw_position.x > x - ASSERV_ERROR_POSITION) and
                (self._robot_raw_position.x < x + ASSERV_ERROR_POSITION) and
                (self._robot_raw_position.y > y - ASSERV_ERROR_POSITION) and
                (self._robot_raw_position.y < y + ASSERV_ERROR_POSITION))

    def _callback_timer_check_reached(self, event):
        rospy.logdebug("In check reached timer callback")
        result = True
        reached = False
        if len(self._last_received_id_dictionary) > 0:
            received_id = self._last_received_id_dictionary.popitem()
            goal_data = received_id[1]
            # Check if the robot is arrived, otherwise this will tell that the robot is blocked (default behaviour of the asserv)
            if len(goal_data) == 2:
                reached = self._check_reached_angle(goal_data[1])
            elif len(goal_data) == 3:
                reached = self._check_reached_position(goal_data[1], goal_data[2])
            elif len(goal_data) == 4:
                reached = self._check_reached_position(goal_data[1], goal_data[2])
                reached &= self._check_reached_angle(goal_data[3])
            else:
                rospy.logwarn("Goal id ack but not corresponding goal data...")
            if not reached:
                rospy.logdebug("Goal has not been reached !")
                result = False
            self._node.goal_reached(goal_data[0], result)
        else:
            rospy.logwarn("Check reached dict empty...")

    def _halt(self):
        """
        This function is kind of a hack, it forces to send the HALT order to the arduino to halt the processing.
        The reason is that some Arduino does not stop when serial communication is shutdown and this make the port_finder not work properly.
        """
        self._send_serial_data(self._orders_dictionary['HALT'], [])
        self._callback_timer_serial_send(rospy.timer.TimerEvent(rospy.Time(), rospy.Time(), rospy.Time(), rospy.Time(), 0.0))
        rospy.logwarn("Send halt to arduino")
