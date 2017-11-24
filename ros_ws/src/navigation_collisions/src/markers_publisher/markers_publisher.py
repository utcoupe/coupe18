#!/usr/bin/python
import math
import rospy
from visualization_msgs.msg import Marker


class MarkersPublisher(object):
    def __init__(self):
        self.MARKERS_TOPIC = "navigation_markers"
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
                self.publishMarker("collisions_path", i, path_shape.Shape, path_shape.Position, 0.02, 0.01, (1.0, 0.2, 0.4, 0.6))

    def publishObstacles(self, obstacles): # Temporaire ?
        for i, obs in enumerate(obstacles):
            self.publishMarker("collisions_obstacles", i, obs.Shape, obs.Position, 0.35, 0.35 / 2.0, (1.0, 0.8, 0.3, 0.8))

    def publishMarker(self, ns, index, shape, position, z_scale, z_height, color):
        markertypes = {
            "rect": Marker.CUBE,
            "circle": Marker.CYLINDER,
            "mesh": Marker.MESH_RESOURCE
        }
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.type = markertypes[str(shape)] # TODO
        marker.ns = ns
        marker.id = index

        marker.action = Marker.ADD
        marker.scale.x = shape.Width  if str(shape) == "rect" else shape.Radius * 2
        marker.scale.y = shape.Height if str(shape) == "rect" else shape.Radius * 2
        marker.scale.z = z_scale
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        marker.pose.position.x = position.X
        marker.pose.position.y = position.Y
        marker.pose.position.z = z_height
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
