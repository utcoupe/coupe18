#!/usr/bin/python
import rospy
import map_manager
import map_communication
from markers import MarkersPublisher
from occupancy import OccupancyGenerator


class MapNode():
    def __init__(self):
        rospy.init_node("map", log_level=rospy.DEBUG)
        rospy.logdebug("Started /memory/map node.")

        map_manager.Map.load()

        # Generate static occupancy images for pathfinder, etc.
        self.occupancy = OccupancyGenerator(map_manager.Map)
        self.occupancy.generateTerrainImages(map_manager.Map)

        # Starting and publishing the table STL to RViz
        self.markers = MarkersPublisher()
        self.markers.publishTable(map_manager.Map)

        # Starting service handlers (Get, Set, Transfer, GetOccupancy)
        map_communication.MapServices()
        rospy.loginfo("[memory/map] Map request servers ready.")

        self.run()

    def run(self):
        r = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.markers.updateMarkers(map_manager.Map)
            r.sleep()


if __name__ == "__main__":
    MapNode()
