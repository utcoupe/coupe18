#!/usr/bin/python
import rospy
import MapMan
import map_communication
from Markers import MarkersPublisher

import time

class MapNode():
    def __init__(self):
        rospy.init_node("map", log_level=rospy.DEBUG)

        s = time.time() * 1000
        MapMan.Map.load("../../def/map_2018.yml")  # TODO to be better when Definitions package available.
        print MapMan.Map.get("/terrain/marker")
        print "loaded map in (ms) " + str(time.time() * 1000 - s)

        # Starting and publishing the table STL to RViz
        self.markers = MarkersPublisher()
        self.markers.publishTable(MapMan.Map)

        # Starting the Get, Set and Conditions service handlers
        map_communication.GetServiceHandler()
        # map_communication.SetServiceHandler()
        # map_communication.ConditionsHandler()
        rospy.loginfo("[memory/map] Map request servers ready.")
        self.run()

    def run(self):
        r = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.markers.publishMarkers(MapMan.Map)
            r.sleep()


if __name__ == "__main__":
    MapNode()
