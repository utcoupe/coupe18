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
from drivers_ard_others.msg import BeltRange
from geometry_msgs.msg import Pose2D, TransformStamped, PointStamped
from memory_map.srv import MapGet


class BeltInterpreter(object):
    def __init__(self):
        super(BeltInterpreter, self).__init__()

        rospy.init_node("belt_interpreter")

        rospy.loginfo("Belt interpreter is initializing...")
        # template for the sensor frame id, with '{}' being the sensor id
        self.SENSOR_FRAME_ID = "belt_{}"
        self.DEF_FILE = "processing/belt.xml"
        self.TOPIC = "/processing/belt_interpreter/rects_filtered"
        self.SENSORS_TOPIC = "/drivers/ard_others/belt_ranges"
        # resolution along the long and large side of the rectangle (meters)
        self.RESOLUTION_LONG = 0.01
        self.RESOLUTION_LARGE = 0.005
        # % the rectangle that need to overlap a map object
        # to be considered static
        self.POINTS_PC_THRESHOLD = 0.5
        self.PUB_RATE = rospy.Rate(20)



        filepath = self.fetch_definition()

        self._belt_parser = BeltParser(filepath)
        self._pub = rospy.Publisher(self.TOPIC, BeltFiltered, queue_size=10)
        self._tl = tf.TransformListener()
        self._broadcaster = tf2_ros.StaticTransformBroadcaster()
        self._markers_pub = MarkersPublisher()

        self.pub_static_transforms()

        self._static_shapes = self.fetch_map_objects()
        self._sensors_sub = rospy.Subscriber(self.SENSORS_TOPIC, BeltRange,
                                             self.callback, queue_size=1)

        self._static_rects = {}
        self._dynamic_rects = {}

        rospy.loginfo("Belt interpreter is ready. Listening for sensor data on '{}'.".format(self.SENSORS_TOPIC))

        self.run_publisher()

    def run_publisher(self):
        while not rospy.is_shutdown():
            self._pub.publish(self._static_rects.values(), self._dynamic_rects.values())
            self._markers_pub.publish_markers(self._static_rects.values(), self._dynamic_rects.values())
            self.PUB_RATE.sleep()

    def callback(self, data):
        # received a range

        sensor_id = data.sensor_id
        if sensor_id not in self._belt_parser.Sensors.keys():
            rospy.logerr("Received data from belt sensor '{}' but no such sensor is defined".format(sensor_id))
            return

        r = data.range
        rospy.loginfo("range value : %f", r)

        if r > self._belt_parser.Params["max_range"] or r <= 0:
            self._static_rects.pop(sensor_id, None)
            self._dynamic_rects.pop(sensor_id, None)
            return

        prec = r * self._belt_parser.Params["precision"]
        angle = self._belt_parser.Params["angle"]

        # define the rectangle in ref to the sensor frame_id
        x_far = r + prec
        x_close = math.cos(angle / 2) * (r - prec)

        # called width because along x axis, but it is the smaller side
        width = abs(x_far - x_close)
        height = abs(2 * math.sin(angle / 2) * (r + prec))

        rect = RectangleStamped()
        rect.header.frame_id = self.SENSOR_FRAME_ID.format(sensor_id)
        ts = self._tl.getLatestCommonTime(rect.header.frame_id, "/map")

        if rospy.Time.now() - ts > rospy.Duration(5):
            rospy.logwarn("Difference between last tf update of /robot and current time is more than 5 seconds. Belt rects may be wrong.")

        rect.header.stamp = ts
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
                pointst.header.stamp = rect.header.stamp

                try:
                    pst_map = self._tl.transformPoint("/map", pointst)
                except Exception as e:
                    rospy.logwarn("Frame robot or map does not exist, cannot process sensor data : {}".format(e))
                    return

                total_points_nbr += 1

                if self.is_static(pst_map):
                    static_points_nbr += 1


        if float(static_points_nbr) / float(total_points_nbr) \
           > self.POINTS_PC_THRESHOLD: #static

            if sensor_id in self._dynamic_rects:
                self._dynamic_rects.pop(sensor_id, None)

            self._static_rects.update({sensor_id: rect})
        else: # dynamic
            if sensor_id in self._static_rects:
                self._static_rects.pop(sensor_id, None)

            self._dynamic_rects.update({sensor_id: rect})

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
                msg = "Can't fetch belt definition file. Shutting down."
                rospy.logfatal(msg)
                raise rospy.ROSInitException(msg)
            else:
                rospy.logdebug("Belt definition file fetched.")
                return res.path

        except rospy.ServiceException as exc:
            msg = "Exception when fetching belt definition file. Shutting down.\n {}".format(str(exc))
            rospy.logfatal(msg)
            raise rospy.ROSInitException(msg)

    def fetch_map_objects(self):
        get_map = rospy.ServiceProxy('/memory/map/get', MapGet)
        get_map.wait_for_service()

        try:
            response = get_map("/terrain/walls/layer_belt/*")

            if not response.success:
                msg ="Can't fetch objects from map. Shutting down."
                rospy.logerr(msg)
                raise rospy.ROSInitException(msg)
            else:
                shapes = []
                map_obj = json.loads(response.response)

                for v in map_obj.values():
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
                        raise NotImplementedError()

                    shapes.append(shape)

                if not shapes:
                    rospy.logwarn("No static objects fetched from map, all sensor data will be treated as dynamic rects.")

                return shapes
        except rospy.ServiceException as exc:
            msg = "Exception when fetching objects from map. Shutting down.\n {}".format(str(exc))
            rospy.logfatal(msg)
            raise rospy.ROSInitException(msg)

    def is_static(self, point_st):
        # need a point stamped in the /map frame
        for shape in self._static_shapes:
            if shape.contains(point_st.point.x, point_st.point.y):
                return True

        return False


if __name__ == '__main__':
    b = BeltInterpreter()
