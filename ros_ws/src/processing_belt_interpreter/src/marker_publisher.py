#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

MARKERS_NAMESPACE = "belt_data"
POINT_SCALE = 0.05
POINT_Z = 0.1

class MarkersPublisher(object):
    def __init__(self):
        super(MarkersPublisher, self).__init__()
        self._pub = rospy.Publisher("/visualization_markers", Marker, queue_size=10)

    def publish_markers(self, static_points, dynamic_points):
        rospy.loginfo("Publishing markers")

        print static_points
        print dynamic_points

        static_points = [p.point for p in static_points]
        dynamic_points = [p.point for p in dynamic_points]

        marker = Marker()
        marker.header.frame_id = "/map"
        marker.header.stamp = rospy.Time.now()
        marker.type = Marker.POINTS
        marker.ns = MARKERS_NAMESPACE
        marker.action = Marker.ADD
        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.pose.position.z = POINT_Z
        marker.color.a = 1
        marker.color.b = 0
        marker.scale.x = POINT_SCALE
        marker.scale.y = POINT_SCALE
        marker.scale.z = POINT_SCALE

        if static_points:
            marker.id = 1
            marker.color.r = 0
            marker.color.g = 1
            marker.points = static_points
            self._pub.publish(marker)

        if dynamic_points:
            marker.id = 2
            marker.color.r = 1
            marker.color.g = 0
            marker.points = dynamic_points
            self._pub.publish(marker)
