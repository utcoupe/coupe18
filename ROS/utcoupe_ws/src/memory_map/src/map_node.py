#!/usr/bin/python
import rospy
import MapManager
import map_communication
from Markers import MarkersPublisher


class MapNode():
    def __init__(self):
        rospy.init_node('map', log_level=rospy.DEBUG)
        MapManager.Map.load("../../def/map_2018.yml")  # TODO to be better when Definitions package available.

        # Starting and publishing the table STL to RViz
        self.markers = MarkersPublisher()
        self.markers.publishTable(MapManager.Map)

        # Starting the Get, Set and Conditions service handlers
        map_communication.GetServiceHandler()
        map_communication.SetServiceHandler()
        map_communication.ConditionsHandler()
        rospy.loginfo("[memory/map] Map request servers ready.")
        self.run()

    def run(self):
        r = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.markers.publishZones(MapManager.Map)
            self.markers.publishObjects(MapManager.Map)
            r.sleep()


if __name__ == "__main__":
    MapNode()
