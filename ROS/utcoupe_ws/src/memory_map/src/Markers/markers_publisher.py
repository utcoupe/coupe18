#!/usr/bin/python
import math
import rospy
from visualization_msgs.msg import Marker


class MarkersPublisher():
    def __init__(self):
        self.RvizConnected = True
        self.MARKERS_TOPIC = "visualization_markers"
        self.MarkersPUBL = rospy.Publisher(self.MARKERS_TOPIC, Marker, queue_size=10)
        
        count = 0
        while not self.MarkersPUBL.get_num_connections():
            # wait for RViz to connect, breaks after a few tries
            rospy.sleep(0.2)
            count += 1
            if count > 5:  # TODO HARDCODED
                self.RvizConnected = False
                break # Cancel connection

    def publishTable(self, mapdict):
        if self.RvizConnected:
            tableSTLMarker = Marker()
            tableSTLMarker.header.frame_id = "/map"
            tableSTLMarker.type = Marker.MESH_RESOURCE
            tableSTLMarker.ns = "table"
            tableSTLMarker.id = 0

            # TODO adapt when Definitions will be done
            tableSTLMarker.mesh_resource = mapdict["terrain"]["mesh_path"]
            tableSTLMarker.action = Marker.ADD
            tableSTLMarker.scale.x = 0.001
            tableSTLMarker.scale.y = 0.001
            tableSTLMarker.scale.z = 0.001
            tableSTLMarker.color.r = 0.8
            tableSTLMarker.color.g = 0.8
            tableSTLMarker.color.b = 0.8
            tableSTLMarker.color.a = 1.0
            tableSTLMarker.pose.position.x = -0.022
            tableSTLMarker.pose.position.y = 0.022 + 2
            tableSTLMarker.pose.position.z = 0.0
            o = self.eulerToQuaternion((1.57079632679, 0, 0))
            tableSTLMarker.pose.orientation.x = o[0]
            tableSTLMarker.pose.orientation.y = o[1]
            tableSTLMarker.pose.orientation.z = o[2]
            tableSTLMarker.pose.orientation.w = o[3]

            self.MarkersPUBL.publish(tableSTLMarker)

    def publishZones(self, mapdict):
        if self.RvizConnected:
            for z in mapdict["terrain"]["zones"]:
                z = mapdict["terrain"]["zones"][z]

                marker = Marker()
                marker.header.frame_id = z.Position.frame_id
                marker.type = z.Visual.Type
                marker.ns = z.Visual.NS
                marker.id = z.Visual.ID
                marker.action = Marker.ADD

                marker.scale.x = z.Visual.Scale[0]
                marker.scale.z = z.Visual.Scale[1]
                marker.scale.y = z.Visual.Scale[2]
                marker.color.r = z.Visual.Color[0]
                marker.color.g = z.Visual.Color[1]
                marker.color.b = z.Visual.Color[2]
                marker.color.a = z.Visual.Color[3]
                marker.pose.position.x = z.Position.x
                marker.pose.position.y = z.Position.y
                marker.pose.position.z = z.Visual.z
                orientation = self.eulerToQuaternion(z.Visual.Orientation)
                marker.pose.orientation.x = orientation[0]
                marker.pose.orientation.y = orientation[1]
                marker.pose.orientation.z = orientation[2]
                marker.pose.orientation.w = orientation[3]

                self.MarkersPUBL.publish(marker)

    def publishObjects(self, mapdict):
        if self.RvizConnected:
            for o in mapdict["objects"]:
                o = mapdict["objects"][o]
                if o.Type == "object":
                    marker = Marker()
                    marker.header.frame_id = o.Position.frame_id
                    marker.type = o.Visual.Type
                    marker.ns = o.Visual.NS
                    marker.id = o.Visual.ID

                    marker.action = Marker.ADD
                    marker.scale.x = o.Visual.Scale[0]
                    marker.scale.z = o.Visual.Scale[1]
                    marker.scale.y = o.Visual.Scale[2]
                    marker.color.r = o.Visual.Color[0]
                    marker.color.g = o.Visual.Color[1]
                    marker.color.b = o.Visual.Color[2]
                    marker.color.a = o.Visual.Color[3]
                    marker.pose.position.x = o.Position.x
                    marker.pose.position.y = o.Position.y
                    marker.pose.position.z = o.Visual.z
                    orientation = self.eulerToQuaternion(o.Visual.Orientation)
                    marker.pose.orientation.x = orientation[0]
                    marker.pose.orientation.y = orientation[1]
                    marker.pose.orientation.z = orientation[2]
                    marker.pose.orientation.w = orientation[3]

                    self.MarkersPUBL.publish(marker)

    def eulerToQuaternion(self, xyz):
        cr = math.cos(xyz[0] * 0.5)
        sr = math.sin(xyz[0] * 0.5)
        cp = math.cos(xyz[1] * 0.5)
        sp = math.sin(xyz[1] * 0.5)
        cy = math.cos(xyz[2] * 0.5)
        sy = math.sin(xyz[2] * 0.5)
        return (cy * sr * cp - sy * cr * sp,  # qx
                cy * cr * sp + sy * sr * cp,  # qy
                sy * cr * cp - cy * sr * sp,  # qz
                cy * cr * cp + sy * sr * sp)  # qw
