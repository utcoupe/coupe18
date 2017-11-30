#!/usr/bin/env python

import rospy
from math import pi, cos, sin
from belt_parser import BeltParser
from shapes import *
from marker_publisher import MarkersPublisher
import tf
import tf2_ros
import json
import math
from numpy import linspace

from memory_definitions.srv import GetDefinition
from processing_belt_interpreter.msg import *
from geometry_msgs.msg import Pose2D, TransformStamped, PointStamped
from memory_map.srv import MapGet


class BeltInterpreter(object):
    def __init__(self):
        super(BeltInterpreter, self).__init__()

        # template for the sensor frame id, with '{}' being the sensor id
        self.SENSOR_FRAME_ID = "belt_{}"
        self.DEF_FILE = "processing/belt.xml"
        self.TOPIC = "/processing/belt_interpreter/rects"
        self.SENSORS_TOPIC = "/sensors/belt"
        # resolution along the long and large side of the rectangle (meters)
        self.RESOLUTION_LONG = 0.01
        self.RESOLUTION_LARGE = 0.005
        # % the rectangle that need to overlap a map object
        # to be considered static
        self.POINTS_PC_THRESHOLD = 0.5

        rospy.init_node("belt_interpreter")

        filepath = self.fetch_definition()

        self._belt_parser = BeltParser(filepath)
        self._pub = rospy.Publisher(self.TOPIC, BeltFiltered, queue_size=10)
        self._tl = tf.TransformListener()
        self._broadcaster = tf2_ros.StaticTransformBroadcaster()
        self._markers_pub = MarkersPublisher()

        self.pub_static_transforms()

        self._static_shapes = self.fetch_map_objects()

        self._sensors_sub = rospy.Subscriber(self.SENSORS_TOPIC, RangeList,
                                             self.callback)

        rospy.spin()

    def callback(self, data_list):
        # received a list of ranges

        static_rects = []
        dynamic_rects = []

        time_stamp = data_list.header.stamp

        for data in data_list.sensors:
            sensor_id = data.sensor_id
            r = data.range
            if r > self._belt_parser.Params["max_range"]:
                continue

            sensor = self._belt_parser.Sensors[sensor_id]

            prec = r * self._belt_parser.Params["precision"]
            angle = self._belt_parser.Params["angle"]

            # define the rectangle in ref to the sensor frame_id
            x_far = r + prec
            x_close = math.cos(angle / 2) * (r - prec)

            # called width because along x axis, but it is the smaller side
            width = x_far - x_close
            height = 2 * math.sin(angle / 2) * (r + prec)

            rect = RectangleStamped()
            rect.header.frame_id = self.SENSOR_FRAME_ID.format(sensor_id)
            rect.header.stamp = time_stamp
            rect.x = (x_far + x_close) / 2
            rect.y = 0
            rect.w = width
            rect.h = height

            static_points_nbr = 0
            total_points_nbr = 0

            for x in linspace(x_close, x_far, width / self.RESOLUTION_LARGE):
                for y in linspace(- height / 2, height / 2,
                                  height / self.RESOLUTION_LONG):

                    pointst = PointStamped()
                    pointst.point.x = x
                    pointst.point.y = y
                    pointst.header = rect.header

                    pst_map = self._tl.transformPoint("map", pointst)

                    total_points_nbr += 1

                    if self.is_static(pst_map):
                        static_points_nbr += 1

            if float(static_points_nbr) / float(total_points_nbr) \
               > self.POINTS_PC_THRESHOLD:
                static_rects.append(rect)
            else:
                dynamic_rects.append(rect)

        self._pub.publish(static_rects, dynamic_rects)
        self._markers_pub.publish_markers(static_rects, dynamic_rects)

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

    def fetch_definition(self):
        get_def = rospy.ServiceProxy('/memory/definitions/get', GetDefinition)
        get_def.wait_for_service()

        try:
            res = get_def(self.DEF_FILE)

            if not res.success:
                rospy.logerr("Error when fetching belt definition file")
                raise Exception()
            else:
                rospy.logdebug("Belt definition file fetched")
                return res.path

        except rospy.ServiceException as exc:
            rospy.logerr("Error when fetching belt definition file: {}"
                         .format(str(exc)))
            raise Exception()

    def fetch_map_objects(self):
        get_map = rospy.ServiceProxy('/memory/map/get', MapGet)
        get_map.wait_for_service()

        response = get_map("/terrain/*")

        if not response.success:
            rospy.logerr("Error when fetching objects from map")
            raise Exception()
        else:
            shapes = []
            map_obj = json.loads(get_map("/terrain/*").response)

            for v in map_obj["walls"]["layer_ground"].values():
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
                else:  # TODO POLYGONS
                    pass

                shapes.append(shape)

            return shapes

    def is_static(self, point_st):
        # need a point stamped in the /map frame
        for shape in self._static_shapes:
            if shape.contains(point_st.point.x, point_st.point.y):
                return True

        return False


if __name__ == '__main__':
    b = BeltInterpreter()
