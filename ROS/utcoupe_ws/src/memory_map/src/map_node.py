#!/usr/bin/python
import rospy
from MapManager import *
from Markers import MarkersPublisher


class MapNode():
    def __init__(self):
        # TODO to be better when Definitions package available.
        rospy.init_node('map', log_level=rospy.DEBUG)
        self.map = Map("../Definitions/map_2018.yml")

        # Starting and publishing the table STL to RViz
        self.markers = MarkersPublisher()
        self.markers.publishTable(self.map.getMapDict())

        # Starting the Get, Set and Conditions service handlers
        GetServiceHandler()
        SetServiceHandler()
        ConditionsHandler()
        self.run()

    def run(self):
        r = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.markers.publishZones(self.map.getMapDict())
            self.markers.publishObjects(self.map.getMapDict())
            r.sleep()


if __name__ == "__main__":
    MapNode()
