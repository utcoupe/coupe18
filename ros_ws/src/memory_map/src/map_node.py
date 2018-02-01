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

        # Starting and publishing the table STL to RViz
        self.markers = MarkersPublisher()

        # Generate static occupancy images for pathfinder, etc.
        occupancy = OccupancyGenerator(map_manager.Map)

        # Starting service handlers (Get, Set, Transfer, GetOccupancy)
        map_communication.MapServices(occupancy)
        rospy.loginfo("[memory/map] Map request servers ready.")

        map_manager.Map.swap_team("orange")

        self.run()

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if rospy.has_param("/current_team"):
                map_manager.Map.CURRENT_TEAM = rospy.get_param("/current_team")
            self.markers.updateMarkers(map_manager.Map)
            r.sleep()


if __name__ == "__main__":
    MapNode()
