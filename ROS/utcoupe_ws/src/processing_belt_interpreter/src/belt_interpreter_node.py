#!/usr/bin/env python

import rospy
from math import pi, cos, sin
from belt_parser import BeltParser
from shapes import *
import tf
import tf2_ros
import json

from memory_definitions.srv import GetDefinition
from processing_belt_interpreter.msg import *
from geometry_msgs.msg import Pose2D, TransformStamped, PointStamped
from memory_map.srv import MapGet


class BeltInterpreter(object):

    def __init__(self):
        super(BeltInterpreter, self).__init__()

        self.SENSOR_FRAME_ID = "belt_{}" # {} will be replaced by the sensor name

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


        self._sensors_sub = rospy.Subscriber("/sensors/belt", RangeList, self.callback)
        self._localizer_sub = rospy.Subscriber("/recognition/localizer", Pose2D, self.callbackPos)
        self._pub = rospy.Publisher("/processing/belt_interpreter/points", BeltFiltered, queue_size=10)
        self._tl = tf.TransformListener()
        self._broadcaster = tf2_ros.StaticTransformBroadcaster()


        self.pub_static_transforms()

        rospy.loginfo("[PROCESSING] belt_interpreter subscribed to sensors topics")

        self._static_shapes = []

        get_map = rospy.ServiceProxy('/memory/map/get', MapGet)
        map = j = json.loads(get_map("/terrain/*"))
        for v in map["walls"]["layer_ground"].values():
            x = float(v["position"]["x"])
            y = float(v["position"]["y"])
            type = v["shape"]["type"]

            if type == "rect":
                w = float(v["shape"]["width"])
                h = float(v["shape"]["height"])
                shape = Rectangle(x, y, w, h)

            elif type == "circle":
                r = float(v["shape"]["radius"])
                shape = Circle(x, y, r)
            else: # TODO POLYGONS
                pass

            self._static_shapes.append(shape)

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
            point.header.frame_id = self.SENSOR_FRAME_ID.format(sensor_id)
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

    def pub_static_transforms(self):
        tr_list = []
        for id, s in self._belt_parser.Sensors.items():
            tr = TransformStamped()

            tr.header.stamp = rospy.Time.now()
            tr.header.frame_id = "robot"
            tr.child_frame_id = self.SENSOR_FRAME_ID.format(id)

            tr.transform.translation.x = s["x"]
            tr.transform.translation.y = s["y"]
            tr.transform.translation.z = 0

            quat = tf.transformations.quaternion_from_euler(0, 0, s["a"])
            tr.transform.rotation.x = quat[0]
            tr.transform.rotation.y = quat[1]
            tr.transform.rotation.z = quat[2]
            tr.transform.rotation.w = quat[3]

            tr_list.append(tr)

        self._broadcaster.sendTransform(tr_list)

if __name__ == '__main__':
    b = BeltInterpreter()
