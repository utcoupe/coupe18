#!/usr/bin/env python

from visualization_msgs.msg import Marker
from random import randint
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler
import rospy

NAMESPACE = "objects_classifed"
TOPIC = "/visualization_markers/objects"
Z = 0.1
SCALE_Z = 0.2

class MarkersPublisher():
    def __init__(self):
        self.ids = []
        self.marker_pub = rospy.Publisher(TOPIC, Marker, queue_size=1)

    def clear_markers(self):
        for id in self.ids:
            m = Marker()
            m.ns = NAMESPACE
            m.action = m.DELETE
            m.id = id
            self.marker_pub.publish(m)
            self.ids.remove(id)




    def publish_rects(self, map_rects, unknown_rects):
        m = Marker()
        m.ns = NAMESPACE
        m.action = m.ADD

        for r in map_rects + unknown_rects:
            m.id = randint(0, 2**10)
            m.header = r.header
            m.type = m.CUBE
            p = Pose()
            p.position.x = r.x
            p.position.y = r.y
            p.position.z = Z
            q = quaternion_from_euler(0, 0, r.a)
            p.orientation.x = q[0]
            p.orientation.y = q[1]
            p.orientation.z = q[2]
            p.orientation.w = q[3]

            m.pose = p
            m.scale.x = r.w
            m.scale.y = r.h
            m.scale.z = SCALE_Z
            m.lifetime = rospy.Duration(0.4)

            m.color.b = 0
            m.color.a = 1

            if r in map_rects:
                m.color.r = 0
                m.color.g = 1
            else:
                m.color.r = 1
                m.color.g = 0

            self.ids.append(m.id)
            self.marker_pub.publish(m)
            rospy.loginfo(self.ids)



