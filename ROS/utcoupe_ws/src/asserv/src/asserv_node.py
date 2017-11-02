#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose2D
import serial
import threading
import Queue
import os
from asserv.srv import *
from asserv.msg import *
import protocol_parser
import check_arduino

__author__ = "Thomas Fuhrmann"
__date__ = 21/10/2017


class Asserv:
    def __init__(self):
        # Internal data members
        self._reception_queue = Queue.Queue()
        self._orders_dictionnary = protocol_parser.protocol_parse(os.environ['UTCOUPE_WORKSPACE'] + "/arduino/common/asserv/protocol.h")
        # Init ROS stuff
        rospy.init_node('asserv', anonymous=False)
        self._pub_robot_pose = rospy.Publisher("robot/pose2d", Pose2D, queue_size=5)
        self._pub_robot_speed = rospy.Publisher("robot/speed", RobotSpeed, queue_size=5)
        # self._sub_arm = rospy.Subscriber("arm", 1, Asserv.callback_arm)
        self._srv_goto = rospy.Service("asserv/controls/goto", Goto, self.callback_goto)
        self._srv_pwm = rospy.Service("asserv/controls/pwm", Pwm, self.callback_pwm)
        self._srv_speed = rospy.Service("asserv/controls/speed", Speed, self.callback_speed)
        self._srv_set_pos = rospy.Service("asserv/controls/set_pos", SetPos, self.callback_set_pos)
        self._srv_emergency_stop = rospy.Service("asserv/controls/emergency_stop", EmergencyStop, self.callback_emergency_stop)
        self._srv_params = rospy.Service("asserv/parameters", Parameters, self.callback_asserv_param)
        self._srv_management = rospy.Service("asserv/management", Management, self.callback_management)
        # Init the serial communication
        self._arduino_startep_flag = False
        self._order_id = 0
        self._serial_com = None
        self._serial_receiver_thread = None
        self.start_serial_com_line(check_arduino.get_arduino_port("asserv"))
        self.start()

    def start_serial_com_line(self, port):
        try:
            self._serial_com = serial.Serial(port, 57600, timeout=0.5)
            self._serial_receiver_thread = threading.Thread(target=self.data_receiver)
            self._serial_receiver_thread.start()
        except serial.SerialException:
            rospy.logerr("Port : " + port + " is not available, make sure you have plugged the right arduino.")

    def callback_arm(self, data):
        rospy.loginfo("ARM callback")

    def callback_goto(self, request):
        response = True
        # TODO manage the direction
        # TODO check the angle
        if request.mode == request.GOTO:
            self.send_serial_data(self._orders_dictionnary['GOTO'], [str(int(request.position.x * 1000)), str(int(request.position.y * 1000)), str(1)])
        elif request.mode == request.GOTOA:
            self.send_serial_data(self._orders_dictionnary['GOTOA'], [str(int(request.position.x * 1000)), str(int(request.position.y * 1000)), str(request.position.theta), str(1)])
        elif request.mode == request.ROT:
            self.send_serial_data(self._orders_dictionnary['ROT'], [str(request.theta)])
        elif request.mode == request.ROTNOMODULO:
            self.send_serial_data(self._orders_dictionnary['ROTNOMODULO'], [str(request.theta)])
        else:
            response = False
            rospy.logerr("GOTO mode %d does not exists...", request.mode)
        return GotoResponse(response)

    def callback_set_pos(self, request):
        # TODO check the angle (rad / degree ?)
        self.send_serial_data(self._orders_dictionnary['SET_POS'], [str(int(request.position.x * 1000)), str(int(request.position.y * 1000)), str(request.position.theta)])
        return SetPosResponse(True)

    def callback_pwm(self, request):
        self.send_serial_data(self._orders_dictionnary['PWM'], [str(request.left), str(request.right), str(request.duration)])
        return PwmResponse(True)

    def callback_speed(self, request):
        self.send_serial_data(self._orders_dictionnary['SPD'], [str(request.linear), str(request.angular), str(request.duration)])
        return SpeedResponse(True)

    def callback_emergency_stop(self, request):
        self.send_serial_data(self._orders_dictionnary['SETEMERGENCYSTOP'], [str(request.enable)])
        return EmergencyStopResponse(True)

    def callback_asserv_param(self, request):
        response = True
        if request.mode == request.SPDMAX:
            self.send_serial_data(self._orders_dictionnary['SPDMAX'], [str(request.spd), str(request.spd_ratio)])
        elif request.mode == request.ACCMAX:
            self.send_serial_data(self._orders_dictionnary['ACCMAX'], [str(request.acc)])
        elif request.mode == request.PIDRIGHT:
            self.send_serial_data(self._orders_dictionnary['PIDRIGHT'], [str(request.p), str(request.i), str(request.d)])
        elif request.mode == request.PIDLEFT:
            self.send_serial_data(self._orders_dictionnary['PIDLEFT'], [str(request.p), str(request.i), str(request.d)])
        elif request.mode == request.PIDALL:
            self.send_serial_data(self._orders_dictionnary['PIDALL'], [str(request.p), str(request.i), str(request.d)])
        else:
            response = False
            rospy.logerr("modes mode %d does not exists...", request.mode)
        return ParametersResponse(response)

    def callback_management(self, request):
        response = True
        if request.mode == request.KILLG:
            self.send_serial_data(self._orders_dictionnary['KILLG'], [])
        elif request.mode == request.CLEANG:
            self.send_serial_data(self._orders_dictionnary['CLEANG'], [])
        elif request.mode == request.PAUSE:
            self.send_serial_data(self._orders_dictionnary['PAUSE'], [])
        elif request.mode == request.RESUME:
            self.send_serial_data(self._orders_dictionnary['RESUME'], [])
        elif request.mode == request.RESET_ID:
            self.send_serial_data(self._orders_dictionnary['RESET_ID'], [])
        else:
            response = False
            rospy.logerr("Management mode %d does not exists...", request.mode)
        return ManagementResponse(response)

    def data_receiver(self):
        while not rospy.is_shutdown():
            try:
                received_data = self._serial_com.readline()
                if received_data != "":
                    # TODO not working well !
                    received_data.replace("\r\n", "")
                    self._reception_queue.put(received_data)
            except KeyboardInterrupt:
                break
            rospy.sleep(0.001)

    def start(self):
        while not rospy.is_shutdown():
            if not self._reception_queue.empty():
                try:
                    self.process_received_data(self._reception_queue.get())
                    self._reception_queue.task_done()
                except KeyboardInterrupt:
                    break
            rospy.sleep(0.001)

    def process_received_data(self, data):
        # rospy.loginfo("Process : " + data)
        # At init, start the Arduino
        if (not self._arduino_startep_flag) and data.find("asserv") != -1:
            self.send_serial_data(self._orders_dictionnary['START'], [])
        elif data.find("~") != -1:
            receied_data_list = data.split(";")
            # rospy.loginfo("data sharp : " + receied_data_list[10])
            # TODO data conversion mm to meters
            self._pub_robot_pose.publish(Pose2D(float(receied_data_list[2]), float(receied_data_list[3]), float(receied_data_list[4])))
            self._pub_robot_speed.publish(RobotSpeed(float(receied_data_list[5]), float(receied_data_list[6]), float(receied_data_list[7]), float(receied_data_list[8]), float(receied_data_list[9])))
        else:
            # Special order ack, the first one concern the Arduino activation
            if data.find("0;") != -1:
                rospy.loginfo("Arduino started")
                self._arduino_startep_flag = True
            # TODO process orders ack reception
            rospy.loginfo("received order ack : %s", data)

    def send_serial_data(self, order_type, args_list):
        if self._serial_com is not None:
            args_list.insert(0, str(self._order_id))
            self._serial_com.write(order_type + ';' + ';'.join(args_list))
            self._order_id += 1


if __name__ == "__main__":
    Asserv()
