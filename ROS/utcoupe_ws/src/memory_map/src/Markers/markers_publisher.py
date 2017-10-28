#!/usr/bin/python
import math
import rospy
from MapManager import map_objects
from visualization_msgs.msg import Marker


class MarkersPublisher():
    def __init__(self):
        self.MARKERS_TOPIC = "visualization_markers"
        self.MarkersPUBL = rospy.Publisher(self.MARKERS_TOPIC, Marker, queue_size=10)

        count = 0  # TODO HARDCODED
        self.RvizConnected = True
        while not self.MarkersPUBL.get_num_connections():  # wait for RViz to connect, breaks after a few tries
            rospy.sleep(0.2)
            count += 1
            if count > 5:
                self.RvizConnected = False
                break # Cancel connection

    def publishTable(self, world):
        if self.RvizConnected:
            pos = map_objects.Position({
                "frame_id": "/map",
                "x": -0.022,
                "y": 0.022 + 2,
                "type": "fixed"
            })
            self.publishMarker(pos, world.Terrain.Visual)
            rospy.logdebug("[memory/map] Published table to RViz.")

    def publishZones(self, world):
        if self.RvizConnected:
            for z in world.Zones.Elements:
                self.publishMarker(z.Position, z.Visual)

    def publishObjects(self, world):
        if self.RvizConnected:
            for o in world.Objects.Elements:
                if o.Type == "object":
                    self.publishMarker(o.Position, o.Visual)
                elif o.Type == "container":
                    rospy.logwarn("Can't show containers yet, not implemented.")

    def publishMarker(self, position, visual):
        marker = Marker()
        marker.header.frame_id = position.frame_id
        marker.type = visual.Type
        marker.ns = visual.NS
        marker.id = visual.ID

        marker.action = Marker.ADD
        marker.scale.x = visual.Scale[0]
        marker.scale.z = visual.Scale[1]
        marker.scale.y = visual.Scale[2]
        marker.color.r = visual.Color[0]
        marker.color.g = visual.Color[1]
        marker.color.b = visual.Color[2]
        marker.color.a = visual.Color[3]
        marker.pose.position.x = position.x
        marker.pose.position.y = position.y
        marker.pose.position.z = visual.z
        orientation = self.eulerToQuaternion(visual.Orientation)
        marker.pose.orientation.x = orientation[0]
        marker.pose.orientation.y = orientation[1]
        marker.pose.orientation.z = orientation[2]
        marker.pose.orientation.w = orientation[3]

        marker.mesh_resource = visual.mesh_path if visual.Type == Marker.MESH_RESOURCE else ''

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
