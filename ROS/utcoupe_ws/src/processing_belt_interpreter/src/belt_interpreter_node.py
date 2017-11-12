#!/usr/bin/env python

import rospy
from math import pi, cos, sin
from belt_parser import BeltParser
import tf
import tf2_ros

from memory_definitions.srv import GetDefinition
from processing_belt_interpreter.msg import *
from geometry_msgs.msg import Pose2D, TransformStamped, PointStamped


class BeltInterpreter(object):
    def __init__(self):
        super(BeltInterpreter, self).__init__()

        rospy.init_node("belt_interpreter")
        rospy.loginfo("[PROCESSING] belt_interpreter node started")

        # get definition file
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

        # parse definition file
        self._belt_parser = BeltParser(def_filename)

        for s in self._belt_parser.Sensors:
            pub_static_transform(s["id"], s["x"], s["y"], s["a"])

        self._sensors_sub = rospy.Subscriber("/sensors/belt", RangeList, self.callback)
        self._localizer_sub = rospy.Subscriber("/recognition/localizer", Pose2D, self.callbackPos)
        self._pub = rospy.Publisher("/processing/belt_interpreter", BeltFiltered, queue_size=10)
        self._tl = tf.TransformListener()

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

            point = PointStamped()
            point.header.stamp = rospy.Time.now()
            point.header.frame_id = "belt_{}".format(sensor_id)
            point.point.x = range

            point_in_map = self._tl.transformPoint("map", point)

            x = point_in_map.point.x
            y = point_in_map.point.y

            isStatic = False
            for s in self._static_shapes:
                if s.contains(x, y):
                    isStatic = True
                    break

            if isStatic:
                staticPoints.append(point_in_map)
            else:
                dynamicPoints.append(point_in_map)

        self._pub.publish("map", staticPoints, dynamicPoints)

    def pub_static_transform(name, x, y, theta):
        broadcaster = tf2_ros.StaticTransformBroadcaster()
        tf = Transform()

        tf.header.stamp = rospy.Time.now()
        tf.header.frame_id = "robot"
        tf.child_frame_id = "belt_{}".format(name)

        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.translation.z = 0

        quat = tf.transformations.quaternion_from_euler(0, 0, theta)
        tf.transform.rotation.x = quat[0]
        tf.transform.rotation.y = quat[1]
        tf.transform.rotation.z = quat[2]
        tf.transform.rotation.w = quat[3]

        broadcaster.sendTransform(static_transformStamped)


if __name__ == '__main__':
    b = BeltInterpreter()
