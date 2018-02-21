#!/usr/bin/env python

import rospy
import tf
import json
from recognition_objects_classifier.msg import *
from memory_map.srv import MapGet
from processing_lidar_objects.msg import Obstacles
from processing_belt_interpreter.msg import BeltRects
from ai_game_status import StatusServices
from shapes import *
from geometry_msgs.msg import PointStamped, PoseStamped
import itertools
from numpy import linspace
from math import atan2

class ObjectsClassifier(object):
    def __init__(self):
        super(ObjectsClassifier, self).__init__()

        rospy.init_node("objects_classifier")

        # resolution along the long and large side of the rectangle (meters)
        self.RESOLUTION_LONG = 0.05
        self.RESOLUTION_LARGE = 0.05

        # if an object velocity x or y is greater than this => unknown
        self.VELOCITY_TRESHOLD = 0.01

        # resolution along the long and large side for rectangles (meters)
        self.RESOLUTION_LONG = 0.05
        self.RESOLUTION_LARGE = 0.05

        self.PUB_TOPIC = "/recognition/objects_classifier/objects"
        self.BELT_TOPIC = "/processing/belt_interpreter/rects"
        self.LIDAR_TOPIC = "/processing/lidar_objects/obstacles"

        # max rect width at which the number of rects samples does not scale anymore (m)
        self.RECT_WIDTH_THRESH = 1

        # % the rectangle that need to overlap a map object
        # to be considered static
        self.POINTS_PC_THRESHOLD = 0.5


        self.PUB_RATE = rospy.Rate(10)

        rospy.loginfo("Objects classifier is initializing...")

        self._pub = rospy.Publisher(self.PUB_TOPIC, ClassifiedObjects, queue_size=1)
        self._belt_sub = rospy.Subscriber(self.BELT_TOPIC, BeltRects, self.belt_callback)
        self._lidar_sub = rospy.Subscriber(self.LIDAR_TOPIC, Obstacles, self.lidar_callback)
        self._tl = tf.TransformListener()
        self._static_shapes, self._map_width, self._map_height = self.fetch_map_objects()

        self._rects = {'map': [], 'unknown': []}
        self._circles = {'map': [], 'unknown': []}
        self._segments = {'map': [], 'unknown': []}
        self._to_process = {'rects': [], 'circles': [], 'segments': []}



        self._static_shapes, self._map_width, self._map_width = self.fetch_map_objects()

        StatusServices("recognition", "objects_classifier").ready(True)
        self.run_publisher()


    def belt_callback(self, data):
        self._to_process['rects'] = data.rects

    def lidar_callback(self, data):

        self._to_process['circles'] = [CircleObstacleStamped(data.header, c) for c in data.circles]
        self._to_process['segments'] = [SegmentObstacleStamped(data.header, s) for s in data.segments]

    def run_publisher(self):
        while not rospy.is_shutdown():
            self.process_data()
            self._pub.publish(self._rects['map'], self._circles['map'], self._segments['map'],
                              self._rects['unknown'], self._circles['unknown'], self._segments['unknown'])

            self.PUB_RATE.sleep()

    def process_data(self):
        self.clear_objects()
        self.process_segments()
        self.process_circles()
        self.process_segments()

    def clear_objects(self):
        self._segments['map'] = []
        self._segments['unknown'] = []
        self._circles['map'] = []
        self._circles['unknown'] = []
        self._rects['map'] = []
        self._rects['unknown'] = []

    def process_segments(self):
        for segment in self._to_process['segments']:
            found_circle = False

            # check if the segments created a circle
            for circle in self._to_process['circles']:
                if self.is_segment_in_circle(segment, circle):
                    found_circle = True
                    self._to_process['circles'].remove(circle)
                    if self.is_circle_static(circle):
                        self._segments['map'].append(segment)
                    else:
                        self._circles['unknown'].append(circle)

            if found_circle:
                continue

            if not self.is_point_static(segment.first_point.x, segment.first_point.y) or not \
                    self.is_point_static(segment.last_point.x, segment.last_point.y):
                self._segments['unknown'].append(segment)
            else:
                self._segments['map'].append(segment)

        self._to_process['segments'] = []

    def process_circles(self):
        for circle in self._to_process['circles']:
            if self.is_circle_static(circle):
                self._circles['unknown'].append(circle)
            else:
                self._circles['map'].append(circle)

        self._to_process['circles'] = []

    def process_rects(self):
        for rect in self._to_process['rects']:
            static_points_nbr = 0
            total_points_nbr = 0

            num_samples_width = int(rect.width / self.RESOLUTION_LARGE)
            num_samples_height = int(rect.height / self.RESOLUTION_LONG)

            if num_samples_width < 2:
                num_samples_width = 2

            if num_samples_height < 2:
                num_samples_height = 2

            for x, y in itertools.product(
                    linspace(rect.x - rect.width / 2, rect.x + rect.width / 2, num_samples_width),
                    linspace(- rect.height / 2, rect.height / 2, num_samples_height)):

                pointst = PointStamped()
                pointst.point.x = x
                pointst.point.y = y
                pointst.header = rect.header

                try:
                    pst_map = self._tl.transformPoint("/map", pointst)
                except Exception as e:
                    rospy.logwarn("Frame robot or map does not exist, cannot process sensor data : {}".format(e))
                    break

                total_points_nbr += 1

                if self.is_point_static(pst_map):
                    static_points_nbr += 1

            posest = PoseStamped()
            posest.pose.position.x = rect.x
            posest.pose.position.y = rect.y
            posest.header = rect.header

            pst_map = self._tl.transformPose("/map", posest)

            rect.x = pst_map.pose.position.x
            rect.y = pst_map.pose.position.y
            rect.header = pst_map.header

            q = pst_map.pose.orientation
            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            rect.a = atan2(siny, cosy)

            if float(static_points_nbr) / float(total_points_nbr) \
                    > self.POINTS_PC_THRESHOLD:  # static
                self._rects['map'].append(rect)
            else:  # dynamic
                self._rects['unknown'].append(rect)

        self._to_process['rects'] = []

    def is_segment_in_circle(self, segment, circle):
        return (segment.segment.first_point.x - circle.circle.center.x) ** 2 + \
        (segment.segment.first_point.y - circle.circle.center.y) ** 2 <= circle.circle.radius ** 2

    def is_circle_static(self, circle):
        return not (circle.circle.velocity.x > self.VELOCITY_TRESHOLD or circle.circle.velocity.y > self.VELOCITY_TRESHOLD \
                or not self.is_point_static(circle.circle.center.x, circle.circle.center.y))

    def is_point_static(self, x, y):
        if x < 0 or x > self._map_width or y < 0 or y > self._map_height:
            return True

        for shape in self._static_shapes:
            if shape.contains(x, y):
                return True

        return False

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

            response = get_map("/terrain/shape/*")

            if not response.success:
                msg ="Can't fetch dims from map. Shutting down."
                rospy.logerr(msg)
                raise rospy.ROSInitException(msg)
            else:
                dims = json.loads(response.response)
                return shape, float(dims["width"]), float(dims["height"])

        except rospy.ServiceException as exc:
            msg = "Exception when fetching objects from map. Shutting down.\n {}".format(str(exc))
            rospy.logfatal(msg)
            raise rospy.ROSInitException(msg)

if __name__ == '__main__':
    ObjectsClassifier()