#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker

MARKERS_NAMESPACE = "belt_data"


class MarkersPublisher(object):
    def __init__(self):
        super(MarkersPublisher, self).__init__()
        self._pub = rospy.Publisher("visualization_markers", Marker, queue_size=10)

    def publish_markers(self, static_points, dynamic_points):
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.type = POINTS
        marker.ns = MARKERS_NAMESPACE
        marker.action = Marker.ADD
        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.color.a = 1
        marker.color.b = 0

        # static points
        marker.id = 0
        marker.color.r = 0
        marker.color.g = 1
        marker.points = static_points
        pub.publish(marker)

        # dynamic points
        marker.id = 1
        marker.color.r = 1
        marker.color.g = 0
        marker.points = dynamic_points
        self._pub.publish(marker)
