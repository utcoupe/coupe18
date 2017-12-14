#!/usr/bin/env python

import serial
import threading
import Queue
import os
import rospy
from geometry_msgs.msg import Pose2D
import actionlib
from drivers_ard_asserv.srv import *
from drivers_ard_asserv.msg import *
import protocol_parser
import check_arduino

__author__ = "Thomas Fuhrmann"
__date__ = 21/10/2017

NODE_NAME = "ard_asserv"
ASSERV_ERROR_POSITION = 0.005  # in meters

class Asserv:
    """
    The Asserv class manages the driver node for communication with the Arduino asserv.
    As this node is used as an interface, it gets orders to send on ROS services. The state of the robot is published on ROS topics.
    This node handles an action (see actionlib) for the Goto movement.
    The communication with the Arduino is made using a serial communication line.
    """
    def __init__(self):
        rospy.logdebug("[ASSERV] Starting asserv_node.")
        # Queue to store the received information from the Arduino
        self._reception_queue = Queue.Queue()
        # A queue is used to send data to prevent to send data too fast, which will result to concatenate two sending and making the Arduino crash
        self._sending_queue = Queue.Queue()
        # Dictionary containing the list of orders which are interpreted by the Arduino (do not modify this dictionary !)
        self._orders_dictionary = protocol_parser.protocol_parse(os.environ['UTCOUPE_WORKSPACE'] + "/arduino/common/asserv/protocol.h")
        # This dictionary stores the goals received by the DoGoto action and which are currently in processing
        self._goals_dictionary = {}
        # This dictionary stores the goals received by the Goto service and which are currently in processing
        self._goto_srv_dictionary = {}
        # Store the current position of robot, this one is the raw value returned by the asserv
        self._robot_raw_position = Pose2D(0, 0, 0)
        # Init ROS stuff
        rospy.init_node(NODE_NAME, anonymous=False, log_level=rospy.INFO)
        self._pub_robot_pose = rospy.Publisher("/drivers/" + NODE_NAME + "/pose2d", Pose2D, queue_size=5)
        self._pub_robot_speed = rospy.Publisher("/drivers/" + NODE_NAME + "/speed", RobotSpeed, queue_size=5)
        # self._sub_arm = rospy.Subscriber("arm", 1, Asserv.callback_arm)
        self._srv_goto = rospy.Service("/drivers/" + NODE_NAME + "/goto", Goto, self.callback_goto)
        self._srv_pwm = rospy.Service("/drivers/" + NODE_NAME + "/pwm", Pwm, self.callback_pwm)
        self._srv_speed = rospy.Service("/drivers/" + NODE_NAME + "/speed", Speed, self.callback_speed)
        self._srv_set_pos = rospy.Service("/drivers/" + NODE_NAME + "/set_pos", SetPos, self.callback_set_pos)
        self._srv_emergency_stop = rospy.Service("/drivers/" + NODE_NAME + "/emergency_stop", EmergencyStop, self.callback_emergency_stop)
        self._srv_params = rospy.Service("/drivers/" + NODE_NAME + "/parameters", Parameters, self.callback_asserv_param)
        self._srv_management = rospy.Service("/drivers/" + NODE_NAME + "/management", Management, self.callback_management)
        self._tmr_serial_send = rospy.Timer(rospy.Duration(0.1), self.callback_timer_serial_send)
        # Note : no cancel callback is used, the reason is that the asserv code does not manage cancelling a specific order
        self._act_goto = actionlib.ActionServer("/drivers/" + NODE_NAME + "/goto_action", DoGotoAction, self.callback_action_goto, auto_start=False)
        # Init the serial communication
        # Flag to tell that the connected Arduino is started : we can send it orders
        self._arduino_started_flag = False
        # The order_id is sent to the asserv to identify the orders, it must be unique
        self._order_id = 0
        # Serial object
        self._serial_com = None
        # Thread dedicated to the reception from the serial line
        self._serial_receiver_thread = None
        self._act_goto.start()
        self.start_serial_com_line(check_arduino.get_arduino_port("asserv"))
        self.start()

    def start_serial_com_line(self, port):
        """
        Initiate the serial communication line.
        @param port:    The port where the Arduino is connected (ex : "/devttyUSB0")
        @type port:     string
        """
        try:
            self._serial_com = serial.Serial(port, 57600, timeout=0.5)
            self._serial_receiver_thread = threading.Thread(target=self.data_receiver)
            self._serial_receiver_thread.start()
            rospy.logdebug("[ASSERV] Serial communication line has started.")
        except serial.SerialException:
            rospy.logerr("[ASSERV] Port : " + port + " is not available, make sure you have plugged the Arduino.")

    def callback_arm(self, data):
        # TODO manage the arm state
        rospy.loginfo("ARM callback")

    def callback_goto(self, request):
        """
        Callback of the Goto service.
        @param request: Service request
        @type request:  GotoRequest
        @return:        True if request has been processed, false otherwise
        @rtype:         GotoResponse
        """
        rospy.logdebug("[ASSERV] Received a request (goto service).")
        # TODO manage the direction
        response = self.process_goto_order(request.mode, request.position.x, request.position.y, request.position.theta)
        if response:
            # TODO store something useful in dictionary ?
            # TODO make it proper...
            self._goto_srv_dictionary[self._order_id - 1] = ""
        else:
            rospy.logerr("[ASSERV] Service GOTO has failed... Mode probably does not exist.")
        return GotoResponse(response)

    def callback_set_pos(self, request):
        """
        Callback of the SetPos service.
        @param request: Service request
        @type request:  SetPosRequest
        @return:        True if request has been processed, false otherwise
        @rtype:         SetPosResponse
        """
        rospy.logdebug("[ASSERV] Received a request (set_pos service).")
        self.send_serial_data(self._orders_dictionary['SET_POS'], [str(int(request.position.x * 1000)), str(int(request.position.y * 1000)), str(int(request.position.theta * 1000.0))])
        return SetPosResponse(True)

    def callback_pwm(self, request):
        """
        Callback of the Pwm service.
        @param request: Service request
        @type request:  PwmRequest
        @return:        True if request has been processed, false otherwise
        @rtype:         PwmResponse
        """
        rospy.logdebug("[ASSERV] Received a request (pwm service).")
        self.send_serial_data(self._orders_dictionary['PWM'], [str(request.left), str(request.right), str(request.duration)])
        return PwmResponse(True)

    def callback_speed(self, request):
        """
        Callback of the Speed service.
        @param request: Service request
        @type request:  SpeedRequest
        @return:        True if request has been processed, false otherwise
        @rtype:         SpeedResponse
        """
        rospy.logdebug("[ASSERV] Received a request (speed service).")
        self.send_serial_data(self._orders_dictionary['SPD'], [str(request.linear), str(request.angular), str(request.duration)])
        return SpeedResponse(True)

    def callback_emergency_stop(self, request):
        """
        Callback of the EmergencyStop service.
        @param request: Service request
        @type request:  EmergencyStopRequest
        @return:        True if request has been processed, false otherwise
        @rtype:         EmergencyStopResponse
        """
        rospy.logdebug("[ASSERV] Received a request (emergency_stop service).")
        self.send_serial_data(self._orders_dictionary['SETEMERGENCYSTOP'], [str(request.enable)])
        return EmergencyStopResponse(True)

    def callback_asserv_param(self, request):
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
            self.send_serial_data(self._orders_dictionary['SPDMAX'], [str(request.spd), str(request.spd_ratio)])
        elif request.mode == request.ACCMAX:
            self.send_serial_data(self._orders_dictionary['ACCMAX'], [str(request.acc)])
        elif request.mode == request.PIDRIGHT:
            self.send_serial_data(self._orders_dictionary['PIDRIGHT'], [str(request.p), str(request.i), str(request.d)])
        elif request.mode == request.PIDLEFT:
            self.send_serial_data(self._orders_dictionary['PIDLEFT'], [str(request.p), str(request.i), str(request.d)])
        elif request.mode == request.PIDALL:
            self.send_serial_data(self._orders_dictionary['PIDALL'], [str(request.p), str(request.i), str(request.d)])
        else:
            response = False
            rospy.logerr("[ASSERV] Parameter mode %d does not exists...", request.mode)
        return ParametersResponse(response)

    def callback_management(self, request):
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
            self.send_serial_data(self._orders_dictionary['KILLG'], [])
        elif request.mode == request.CLEANG:
            self.send_serial_data(self._orders_dictionary['CLEANG'], [])
            # Delete all internal goals
            for goal in self._goals_dictionary:
                goal.set_canceled()
            self._goals_dictionary.clear()
        elif request.mode == request.PAUSE:
            self.send_serial_data(self._orders_dictionary['PAUSE'], [])
        elif request.mode == request.RESUME:
            self.send_serial_data(self._orders_dictionary['RESUME'], [])
        elif request.mode == request.RESET_ID:
            self.send_serial_data(self._orders_dictionary['RESET_ID'], [])
        else:
            response = False
            rospy.logerr("[ASSERV] Management mode %d does not exists...", request.mode)
        return ManagementResponse(response)

    def callback_action_goto(self, goal_handled):
        """
        Callback of the DoGoto action.
        @param goal_handled:    Goal handler corresponding to the received action
        @type goal_handled:     ServerGoalHandle
        """
        rospy.logdebug("[ASSERV] Received a request (dogoto action).")
        if self.process_goto_order(goal_handled.get_goal().mode, goal_handled.get_goal().position.x, goal_handled.get_goal().position.y, goal_handled.get_goal().position.theta):
            goal_handled.set_accepted()
            # TODO make it proper...
            self._goals_dictionary[self._order_id - 1] = goal_handled
        else:
            rospy.logerr("[ASSERV] Action GOTO has failed... Mode probably does not exist.")

    def data_receiver(self):
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

    def start(self):
        """
        Starts the node, process the reception queue.
        """
        rospy.logdebug("[ASSERV] Node has correctly started.")
        while not rospy.is_shutdown():
            if not self._reception_queue.empty():
                try:
                    self.process_received_data(self._reception_queue.get())
                    self._reception_queue.task_done()
                except KeyboardInterrupt:
                    break
            rospy.sleep(0.01)

    def process_received_data(self, data):
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
            # rospy.loginfo("data sharp : " + receied_data_list[10])
            robot_position = Pose2D(float(receied_data_list[2]) / 1000, float(receied_data_list[3]) / 1000, float(receied_data_list[4]) / 1000)
            self._robot_raw_position = robot_position
            self._pub_robot_pose.publish(robot_position)
            self._pub_robot_speed.publish(RobotSpeed(float(receied_data_list[5]), float(receied_data_list[6]), float(receied_data_list[7]), float(receied_data_list[8]), float(receied_data_list[9])))
        # Received order ack
        elif data.find(";") == 1:
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
                if ack_id in self._goals_dictionary:
                    rospy.logdebug("[ASSERV] Found key %d in goal dictionary !", ack_id)
                    rospy.loginfo("robot x : %f, goal x : %f", self._robot_raw_position.x, self._goals_dictionary[ack_id].get_goal().position.x)
                    result = DoGotoResult(True)
                    # Check if the robot is arrived, otherwise this will tell that the robot is blocked (default behaviour of the asserv)
                    if not ((self._robot_raw_position.x > self._goals_dictionary[ack_id].get_goal().position.x - ASSERV_ERROR_POSITION) and
                            (self._robot_raw_position.x < self._goals_dictionary[ack_id].get_goal().position.x + ASSERV_ERROR_POSITION) and
                            (self._robot_raw_position.y > self._goals_dictionary[ack_id].get_goal().position.y - ASSERV_ERROR_POSITION) and
                            (self._robot_raw_position.y < self._goals_dictionary[ack_id].get_goal().position.y + ASSERV_ERROR_POSITION)):
                        rospy.loginfo("Goal has not been reached !")
                        result.result = False
                    self._goals_dictionary[ack_id].set_succeeded(result)
                    del self._goals_dictionary[ack_id]
                elif ack_id in self._goto_srv_dictionary:
                    rospy.logdebug("[ASSERV] Found key %d in goto dictionary !", ack_id)
                    # TODO something else ?
                    del self._goto_srv_dictionary[ack_id]
                else:
                    # Do nothing, some IDs are returned but do not correspond to a value in the dictionary.
                    pass
        else:
            rospy.loginfo("%s", data)

    def send_serial_data(self, order_type, args_list):
        """
        This function sends data to the Arduino, using the specific protocol.
        @param order_type:  Type of the order to send (see protocol.h in arduino/common/asserv folder)
        @type order_type:   string containing an int
        @param args_list:   List of arguments, depending on the order
        @type args_list:    List
        """
        if self._serial_com is not None:
            args_list.insert(0, str(self._order_id))
            # TODO check if \n is necessary  + '\n'
            self._sending_queue.put(order_type + ";" + ";".join(args_list) + ";\n")
            self._order_id += 1
        else:
            rospy.logerr("[ASSERV] Try to send data but serial line is not connected...")

    def process_goto_order(self, mode, x, y, a):
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
            self.send_serial_data(self._orders_dictionary['GOTO'], [str(int(x * 1000)), str(int(y * 1000)), str(1)])
        elif mode == GotoRequest.GOTOA:
            self.send_serial_data(self._orders_dictionary['GOTOA'], [str(int(x * 1000)), str(int(y * 1000)), str(a * 1000), str(1)])
        elif mode == GotoRequest.ROT:
            self.send_serial_data(self._orders_dictionary['ROT'], [str(a * 1000)])
        elif mode == GotoRequest.ROTNOMODULO:
            self.send_serial_data(self._orders_dictionary['ROTNOMODULO'], [str(a * 1000)])
        else:
            to_return = False
            rospy.logerr("[ASSERV] GOTO mode %d does not exists...", mode)
        return to_return

    def callback_timer_serial_send(self, event):
        if not self._sending_queue.empty():
            data_to_send = self._sending_queue.get()
            rospy.logdebug("Sending data : " + data_to_send)
            self._serial_com.write(data_to_send)
            self._sending_queue.task_done()


if __name__ == "__main__":
    Asserv()
