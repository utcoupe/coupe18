#!/usr/bin/env python

import rospy
from math import pi, cos, sin
from belt_parser import BeltParser
from memory_definitions.srv import GetDefinition
from processing_belt_interpreter.msg import *
from geometry_msgs.msg import Pose2D, Point32
from functools import partial

# TODO: use tf

class BeltInterpreter(object):
    def __init__(self):
        super(BeltInterpreter, self).__init__()

        rospy.init_node("belt_interpreter")
        rospy.loginfo("[PROCESSING] belt_interpreter node started")

        rospy.wait_for_service('/memory/definitions')
        get_def = rospy.ServiceProxy('/memory/definitions', GetDefinition)

        try:
            res = get_def("processing/Belt.xml")
            if not res.success:
                rospy.logerr("[PROCESSING] Error when fetching belt definition file")

            def_filename = res.path
        except rospy.ServiceException as exc:
            rospy.logerr("[PROCESSING] Error when fetching belt definition file: {}"
                         .format(str(exc)))
            raise Exception()

        rospy.loginfo("[PROCESSING] Belt definition file fetched")

        self._belt_parser = BeltParser(def_filename)

        self._sensors_sub = rospy.Subscriber("/sensors/belt", RangeList, self.callback)
        self._localizer_sub = rospy.Subscriber("/recognition/localizer", Pose2D, self.callbackPos)
        self._pub = rospy.Publisher("/processing/belt_interpreter", BeltFiltered, queue_size=10)

        rospy.loginfo("[PROCESSING] belt_interpreter subscribed to sensors topics")

        self._static_shapes = []
        # TODO: fetch all map shapes

        self._robot_pos = None
        rospy.spin()

    def callbackPos(self, data):
        self._robot_pos = data

    def callback(self, data_list):
        if self._robot_pos is None:
            rospy.logdebug("[PROCESSING] belt_interpreter : Sensor data received but no position received yet from localizer")
            return

        staticPoints = []
        dynamicPoints = []

        for data in data_list.sensors:
            sensor_id = data.sensor_id

            range = data.range
            sensor = self._belt_parser.Sensors[sensor_id]

            angle = sensor["a"] * pi / 180

            #  absolute pos of sensor
            s_x = self._robot_pos.x + cos(self._robot_pos.theta)*sensor["x"] \
                - sin(self._robot_pos.theta)*sensor["y"]
            s_y = self._robot_pos.y + cos(self._robot_pos.theta)*sensor["y"] \
                + sin(self._robot_pos.theta)*sensor["x"]

            dx = range * cos(angle + self._robot_pos.theta)
            dy = range * sin(angle + self._robot_pos.theta)

            x = s_x + dx
            y = s_y + dy

            point = Point32(x, y, 0)

            isStatic = False
            for s in self._static_shapes:
                if s.contains(x, y):
                    isStatic = True
                    break

            if isStatic:
                staticPoints.append(point)
            else:
                dynamicPoints.append(point)

        self._pub.publish("/map", staticPoints, dynamicPoints)


if __name__ == '__main__':
    b = BeltInterpreter()
