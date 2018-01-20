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
        rospy.logdebug("[memory/map] Map request servers ready.")

        status_services = self._get_status_services("memory", "map")
        status_services.ready(True) # Tell ai/game_status the node initialized successfuly.

        self.run()

    def _get_status_services(self, ns, node_name, arm_cb=None, status_cb=None):
        import sys, os
        sys.path.append(os.environ['UTCOUPE_WORKSPACE'] + '/ros_ws/src/ai_game_status/')
        from init_service import StatusServices
        return StatusServices(ns, node_name, arm_cb, status_cb)

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.markers.updateMarkers(map_manager.Map)
            r.sleep()


if __name__ == "__main__":
    MapNode()
