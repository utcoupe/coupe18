#!/usr/bin/python
import math
import rospy
from visualization_msgs.msg import Marker


class MarkersPublisher(object):
    def __init__(self):
        self.MARKERS_TOPIC = "visualization_markers"
        self.MarkersPUBL = rospy.Publisher(self.MARKERS_TOPIC, Marker, queue_size=10)

        count = 0  # TODO HARDCODED
        self.RvizConnected = True
        while not self.MarkersPUBL.get_num_connections():  # wait for RViz to connect, breaks after a few tries
            rospy.sleep(0.2)
            count += 1
            if count > 3:
                rospy.logwarn("WARNING RViz not detected. Map won't publish markers.")
                self.RvizConnected = False
                break # Cancel connection
        if self.RvizConnected: rospy.loginfo("Map connected to RViz. Will publish markers.")

    def publishPathShapes(self, robot):
        if self.RvizConnected:
            # Publish path collision shapes
            for i, path_shape in enumerate(robot.Path.toShapes(robot)):
                self.publishPathShape(path_shape.Position, path_shape.Shape, i)

    def publishPathShape(self, position, shape, i):
        markertypes = {
            "cube": Marker.CUBE,
            "sphere": Marker.SPHERE,
            "mesh": Marker.MESH_RESOURCE
        }
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.type = markertypes["cube"]
        marker.ns = "path_collisions"
        marker.id = i

        marker.action = Marker.ADD
        marker.scale.x = shape.Width
        marker.scale.y = shape.Height
        marker.scale.z = 0.02
        marker.color.r = 1.0
        marker.color.g = 0.2
        marker.color.b = 0.4
        marker.color.a = 0.6
        marker.pose.position.x = position.X
        marker.pose.position.y = position.Y
        marker.pose.position.z = 0.01
        orientation = self.eulerToQuaternion([0, 0, position.A])
        marker.pose.orientation.x = orientation[0]
        marker.pose.orientation.y = orientation[1]
        marker.pose.orientation.z = orientation[2]
        marker.pose.orientation.w = orientation[3]

        self.MarkersPUBL.publish(marker)

    def eulerToQuaternion(self, xyz):
        cr, sr = math.cos(xyz[0] * 0.5), math.sin(xyz[0] * 0.5)
        cp, sp = math.cos(xyz[1] * 0.5), math.sin(xyz[1] * 0.5)
        cy, sy = math.cos(xyz[2] * 0.5), math.sin(xyz[2] * 0.5)
        return (cy * sr * cp - sy * cr * sp,  # qx
                cy * cr * sp + sy * sr * cp,  # qy
                sy * cr * cp - cy * sr * sp,  # qz
                cy * cr * cp + sy * sr * sp)  # qw
