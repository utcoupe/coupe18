#!/usr/bin/env python

import rospy
from math import pi, cos, sin
from belt_parser import BeltParser
from definitions.srv import GetDefinition
from belt_interpreter.msg import *
from geometry_msgs.msg import Pose2D
from functools import partial


class BeltInterpreter(object):
    def __init__(self):
        super(BeltInterpreter, self).__init__()

        rospy.init_node("belt_interpreter")
        rospy.loginfo("[RECOGNITION] belt_interpreter node started")

        rospy.wait_for_service('/memory/definitions')
        get_def = rospy.ServiceProxy('/memory/definitions', GetDefinition)

        try:
            res = get_def("recognition", "belt")
            if not res.success:
                rospy.logerr("[RECOGNITION] Error when fetching belt definition file")

            def_filename = res.path
        except rospy.ServiceException as exc:
            rospy.logerr("[RECOGNITION] Error when fetching belt definition file: {}"
                         .format(str(exc)))

        rospy.loginfo("[RECOGNITION] Belt definition file fetched")

        self.BeltParser = BeltParser(def_filename)
        self.Topics = []

        for s in self.BeltParser.Sensors:
            self.Topics.append(rospy.Subscriber("/sensors/sensor_{}".format(s),
                               Range, partial(self.callback, s)))

        rospy.Subscriber("/recognition/localizer", Pose2D, self.callbackPos)

        rospy.loginfo("[RECOGNITION] belt_interpreter subscribed to sensors topics")

        self.StaticShapes = []
        # TODO: fetch all map shapes

        self.RobotPos = None
        rospy.spin()

    def callbackPos(self, data):
        self.RobotPos = data

    def callback(self, sensor_id, data):
        if self.RobotPos is None:
            rospy.logdebug("[RECOGNITION] belt_interpreter : Sensor data received but no position received yet from localizer")
            return

        if data.sensor_id != sensor_id:
            rospy.logerr("[RECOGNITION] Belt received wrong sensor id for this topic !")

        range = data.range
        sensor = self.BeltParser.Sensors[sensor_id]

        angle = sensor["a"] * pi / 180

        #  absolute pos of sensor
        s_x = self.RobotPos.x + cos(self.RobotPos.theta)*sensor["x"] \
            - sin(self.RobotPos.theta)*sensor["y"]
        s_y = self.RobotPos.y + cos(self.RobotPos.theta)*sensor["y"] \
            + sin(self.RobotPos.theta)*sensor["x"]

        dx = range * cos(angle + self.RobotPos.theta)
        dy = range * sin(angle + self.RobotPos.theta)

        x = s_x + dx
        y = s_y + dy

        rospy.loginfo("Received sensor data, absolute coords : {};{}".format(x, y))



if __name__ == '__main__':
    b = BeltInterpreter()
