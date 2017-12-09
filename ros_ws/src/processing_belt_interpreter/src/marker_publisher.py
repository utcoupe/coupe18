#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker
import hashlib

MARKERS_NAMESPACE = "belt_data"
POINT_SCALE = 0.05
POINT_Z = 0.1

class MarkersPublisher(object):
    def __init__(self):
        super(MarkersPublisher, self).__init__()
        self._pub = rospy.Publisher("/visualization_markers", Marker, queue_size=10)

    def publish_markers(self, static_rects, dynamic_rects): # lists of (id, rect)
        rospy.loginfo("Publising markers")
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.header.stamp = rospy.Time.now()
        marker.type = Marker.CUBE
        marker.ns = MARKERS_NAMESPACE
        marker.action = Marker.ADD
        marker.pose.position.z = POINT_Z
        marker.color.a = 1
        marker.color.b = 0
        marker.scale.x = POINT_SCALE
        marker.scale.y = POINT_SCALE
        marker.scale.z = POINT_SCALE


        for id, rect in static_rects:
            id_hash = int(hashlib.md5(id).hexdigest(), 16)

            marker.id = id_hash
            marker.color.r = 0
            marker.color.g = 1
            marker.scale.x = rect.w
            marker.scale.y = rect.h
            marker.pose.position.x = rect.x
            marker.pose.position.y = rect.y
            self._pub.publish(marker)

        for id, rect in dynamic_rects:
            id_hash = int(hashlib.md5(id).hexdigest(), 16)

            marker.id = id_hash
            marker.color.r = 1
            marker.color.g = 0
            marker.scale.x = rect.w
            marker.scale.y = rect.h
            marker.pose.position.x = rect.x
            marker.pose.position.y = rect.y
            self._pub.publish(marker)
