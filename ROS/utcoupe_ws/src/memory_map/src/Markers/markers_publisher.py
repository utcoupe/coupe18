#!/usr/bin/python
import math
import rospy
from MapMan import map_attributes
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
                rospy.logwarn("WARNING RViz not detected. Map won't publish markers.")
                self.RvizConnected = False
                break # Cancel connection

    def publishTable(self, world):
        if self.RvizConnected:
            pos = map_attributes.Position2D({
                "frame_id": "/map",
                "x": -0.022,
                "y": 0.022 + 2,
                "type": "fixed"
            })
            self.publishMarker(pos, world.get("/terrain/marker"))
            rospy.logdebug("[memory/map] Published table to RViz.")

    def publishMarkers(self, world):
        self.publishZones(world)
        self.publishObjects(world)

    def publishZones(self, world):
        if self.RvizConnected:
            for z in world.get("zones").toList():
                self.publishMarker(z.get("position"), z.get("marker"))

    def publishObjects(self, world):
        if self.RvizConnected:
            for o in world.get("objects").toList():
                if o.get("type") == "object":
                    self.publishMarker(o.get("position"), o.get("marker"))
                elif o.get("type") == "container":
                    for e in o.toList():
                        self.publishMarker(e.get("position"), e.get("marker")) # TODO CAUTION can't show objects in a container in a container yet #23h

    def publishMarker(self, position, visual):
        markertypes = {
            "cube": Marker.CUBE,
            "sphere": Marker.SPHERE,
            "mesh": Marker.MESH_RESOURCE
        }
        marker = Marker()
        marker.header.frame_id = position.get("frame_id")
        marker.type = markertypes[visual.get("type")]
        marker.ns = visual.get("ns")
        marker.id = visual.get("id")

        marker.action = Marker.ADD
        marker.scale.x = visual.get("scale")[0]
        marker.scale.z = visual.get("scale")[1]
        marker.scale.y = visual.get("scale")[2]
        marker.color.r = visual.get("color")[0]
        marker.color.g = visual.get("color")[1]
        marker.color.b = visual.get("color")[2]
        marker.color.a = visual.get("color")[3]
        marker.pose.position.x = position.get("x")
        marker.pose.position.y = position.get("y")
        marker.pose.position.z = visual.get("z")
        orientation = self.eulerToQuaternion(visual.get("orientation"))
        marker.pose.orientation.x = orientation[0]
        marker.pose.orientation.y = orientation[1]
        marker.pose.orientation.z = orientation[2]
        marker.pose.orientation.w = orientation[3]

        marker.mesh_resource = visual.get("mesh_path") if marker.type == Marker.MESH_RESOURCE else ''

        self.MarkersPUBL.publish(marker)

    def eulerToQuaternion(self, xyz):
        cr, sr = math.cos(xyz[0] * 0.5), math.sin(xyz[0] * 0.5)
        cp, sp = math.cos(xyz[1] * 0.5), math.sin(xyz[1] * 0.5)
        cy, sy = math.cos(xyz[2] * 0.5), math.sin(xyz[2] * 0.5)
        return (cy * sr * cp - sy * cr * sp,  # qx
                cy * cr * sp + sy * sr * cp,  # qy
                sy * cr * cp - cy * sr * sp,  # qz
                cy * cr * cp + sy * sr * sp)  # qw
