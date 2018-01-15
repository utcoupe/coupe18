#!/usr/bin/python
import math
import rospy
from visualization_msgs.msg import Marker


class MarkersPublisher(object):
    def __init__(self):
        self.MARKERS_TOPIC = "/visualization_markers/navigation"
        self.MarkersPUBL = rospy.Publisher(self.MARKERS_TOPIC, Marker, queue_size=10)

    def _is_connected(self):
        return bool(self.MarkersPUBL.get_num_connections())

    def publishCheckZones(self, robot):
        if self._is_connected():
            # Publish path collision shapes
            if robot.isInitialized() and robot.NavStatus:
                path_shapes = robot.Path.toShapes(robot)
                if len(path_shapes) > 0:
                    for i, path_shape in enumerate(path_shapes):
                        self._publish_marker("collisions_path", i + 1, path_shape, 0.02, 0.01, (1.0, 0.5, 0.1, 0.8))
                stop_rect = robot.getStopRect()
                self._publish_marker("collisions_path", 0, stop_rect, 0.02, 0.01, (1.0, 0.0, 0.0, 0.8))

    def publishObstacles(self, obstacles): # Temporaire ?
        if self._is_connected():
            for i, obs in enumerate(obstacles):
                self._publish_marker("collisions_obstacles", i, obs, 0.35, 0.35 / 2.0, (1.0, 0.8, 0.3, 0.8))

    def _publish_marker(self, ns, index, obj, z_scale, z_height, color):
        markertypes = {
            "rect": Marker.CUBE,
            "circle": Marker.CYLINDER,
            "mesh": Marker.MESH_RESOURCE
        }
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.type = markertypes[str(obj)] # TODO
        marker.ns = ns
        marker.id = index

        marker.action = Marker.ADD
        marker.scale.x = obj.Width  if str(obj) == "rect" else obj.Radius * 2
        marker.scale.y = obj.Height if str(obj) == "rect" else obj.Radius * 2
        marker.scale.z = z_scale
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        marker.pose.position.x = obj.Position.X
        marker.pose.position.y = obj.Position.Y
        marker.pose.position.z = z_height
        orientation = self._euler_to_quaternion([0, 0, obj.Position.A])
        marker.pose.orientation.x = orientation[0]
        marker.pose.orientation.y = orientation[1]
        marker.pose.orientation.z = orientation[2]
        marker.pose.orientation.w = orientation[3]

        self.MarkersPUBL.publish(marker)

    def _euler_to_quaternion(self, xyz):
        cr, sr = math.cos(xyz[0] * 0.5), math.sin(xyz[0] * 0.5)
        cp, sp = math.cos(xyz[1] * 0.5), math.sin(xyz[1] * 0.5)
        cy, sy = math.cos(xyz[2] * 0.5), math.sin(xyz[2] * 0.5)
        return (cy * sr * cp - sy * cr * sp,  # qx
                cy * cr * sp + sy * sr * cp,  # qy
                sy * cr * cp - cy * sr * sp,  # qz
                cy * cr * cp + sy * sr * sp)  # qw
