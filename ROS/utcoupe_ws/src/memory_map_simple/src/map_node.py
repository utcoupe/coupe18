#!/usr/bin/python
import rospy
from map import Map
from map_communication import *
# from Markers import MarkersPublisher


class MapNode():
    def __init__(self):
        # TODO to be better when Definitions package available.
        rospy.init_node('map', log_level=rospy.DEBUG)
        Map.load("/Definitions/map_2018.yml")

        # Starting and publishing the table STL to RViz
        # self.markers = MarkersPublisher()
        # self.markers.publishTable(Map.getMapDict())

        # Starting the Get, Set and Conditions service handlers
        GetServiceHandler()
        SetServiceHandler()
        ConditionsHandler()
        self.run()

    def run(self):
        r = rospy.Rate(30)
        while not rospy.is_shutdown():
            # self.markers.publishZones(Map.getMapDict())
            # self.markers.publishObjects(Map.getMapDict())
            r.sleep()


if __name__ == "__main__":
    MapNode()
