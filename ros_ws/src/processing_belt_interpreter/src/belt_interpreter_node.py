#!/usr/bin/env python

import rospy
from math import pi, cos, sin
from belt_parser import BeltParser
from shapes import *
from marker_publisher import MarkersPublisher
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

        self.SENSOR_FRAME_ID = "belt_{}"  # {} will be replaced by the sensor name
        self.EXTRAPOLATION_MARGIN_MAX = 0.1  # if the difference of time beetween the last transform available and the sensor data reception is larger than this value (sec), a warning is raised

        rospy.init_node("belt_interpreter")
        rospy.loginfo("Node started")

        # get definition file
        get_def = rospy.ServiceProxy('/memory/definitions/get', GetDefinition)
        get_def.wait_for_service()
        try:
            res = get_def("processing/belt.xml")
            if not res.success:
                rospy.logerr("Error when fetching belt definition file")

            def_filename = res.path
        except rospy.ServiceException as exc:
            rospy.logerr("Error when fetching belt definition file: {}"
                         .format(str(exc)))
            raise Exception()

        rospy.loginfo("Belt definition file fetched")

        # parse definition file
        self._belt_parser = BeltParser(def_filename)

        self._pub = rospy.Publisher("/processing/belt_interpreter/points", BeltFiltered, queue_size=10)
        self._tl = tf.TransformListener()
        self._broadcaster = tf2_ros.StaticTransformBroadcaster()
        self._markers_pub = MarkersPublisher()

        self.pub_static_transforms()

        self._static_shapes = []

        rospy.wait_for_service('/memory/map/get')
        get_map = rospy.ServiceProxy('/memory/map/get', MapGet)
        response = get_map("/terrain/*")
        if not response.success:
            rospy.logerr("Error when fetching objects from map")
        else:
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
                else: # TODO POLYGONS
                    pass

                self._static_shapes.append(shape)

        self._sensors_sub = rospy.Subscriber("/sensors/belt", RangeList, self.callback)
        rospy.loginfo("Subscribed to sensors topics")

        rospy.spin()

    def callback(self, data_list):
        static_points = []
        dynamic_points = []
        for data in data_list.sensors:
            sensor_id = data.sensor_id

            range = data.range
            sensor = self._belt_parser.Sensors[sensor_id]

            point = PointStamped()
            point.header.frame_id = self.SENSOR_FRAME_ID.format(sensor_id)

            try:
                diff = rospy.Time.now() - self._tl.getLatestCommonTime(point.header.frame_id, "map")
            except tf2_rosLookupException as e:
                rospy.logerr("Frame 'map' does not exist, cannot process points\n{}".format(e))
                return

            if(diff.to_sec() > self.EXTRAPOLATION_MARGIN_MAX):
                rospy.logwarn("The difference between the last stored transform of {} and the current time is larger than {} ({}), this may cause errors".format(point.header.frame_id, self.EXTRAPOLATION_MARGIN_MAX, diff.to_sec()))

            point.header.stamp = self._tl.getLatestCommonTime(point.header.frame_id, "map")

            point.point.x = range

            try:
                point_in_map = self._tl.transformPoint("map", point)
            except tf2_ros.LookupException as e:
                rospy.logerr("Frame 'map' does not exist, cannot process points\n{}".format(e))
                return

            x = point_in_map.point.x
            y = point_in_map.point.y

            isStatic = False
            for s in self._static_shapes:
                if s.contains(x, y):
                    isStatic = True
                    break

            if isStatic:
                static_points.append(point_in_map)
            else:
                dynamic_points.append(point_in_map)

        self._pub.publish("map", static_points, dynamic_points)
        self._markers_pub.publish_markers(static_points, dynamic_points)

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
