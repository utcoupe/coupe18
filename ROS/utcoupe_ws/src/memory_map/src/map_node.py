#!/usr/bin/python
import rospy
import MapManager
from Markers import MarkersPublisher


class MapNode():
    def __init__(self):
        rospy.init_node('map', log_level=rospy.DEBUG)
        self.World = MapManager.Map("../Definitions/map_2018.yml")  # TODO to be better when Definitions package available.

        # Starting and publishing the table STL to RViz
        self.markers = MarkersPublisher()
        self.markers.publishTable(self.World)

        # Starting the Get, Set and Conditions service handlers
        MapManager.GetServiceHandler()
        MapManager.SetServiceHandler()
        MapManager.ConditionsHandler()
        self.run()

    def run(self):
        r = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.markers.publishZones(self.World)
            self.markers.publishObjects(self.World)
            r.sleep()


if __name__ == "__main__":
    MapNode()
