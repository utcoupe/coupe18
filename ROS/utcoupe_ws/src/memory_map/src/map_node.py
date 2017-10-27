#!/usr/bin/python
import rospy
from MapManager import *
from Markers import MarkersPublisher


class MapNode():
    def __init__(self):
        # TODO to be better when Definitions package available.
        rospy.init_node('map', log_level=rospy.DEBUG)
        world = Map("../Definitions/map_2018.yml")
        print world.Objects.Elements[0].Position.x # Temp test for accessing values

        # Starting and publishing the table STL to RViz
        self.markers = MarkersPublisher()
        self.markers.publishTable(Map.getMapDict())

        # Starting the Get, Set and Conditions service handlers
        GetServiceHandler()
        SetServiceHandler()
        ConditionsHandler()
        self.run()

    def run(self):
        r = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.markers.publishZones(Map.getMapDict())
            self.markers.publishObjects(Map.getMapDict())
            r.sleep()


if __name__ == "__main__":
    MapNode()
