#!/usr/bin/python
import rospy
import MapManager
import map_communication
from Markers import MarkersPublisher
from Occupancy import OccupancyGenerator


class MapNode():
    def __init__(self):
        rospy.init_node("map", log_level=rospy.DEBUG)
        rospy.logdebug("Started /memory/map node.")

        MapManager.Map.load()

        # Generate static occupancy images for pathfinder, etc.
        self.occupancy = OccupancyGenerator(MapManager.Map)
        self.occupancy.generateTerrainImages(MapManager.Map)

        # Starting and publishing the table STL to RViz
        self.markers = MarkersPublisher()
        self.markers.publishTable(MapManager.Map)

        # Starting the Get, Set and Conditions service handlers
        map_communication.GetServiceHandler()
        map_communication.SetServiceHandler()
        rospy.loginfo("[memory/map] Map request servers ready.")

        self.run()

    def run(self):
        r = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.markers.updateMarkers(MapManager.Map)
            r.sleep()


if __name__ == "__main__":
    MapNode()
