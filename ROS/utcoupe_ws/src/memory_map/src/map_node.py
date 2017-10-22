#!/usr/bin/python
import rospy
from MapManager import MapLoader, GetServiceHandler, SetServiceHandler
from Markers import MarkersPublisher


class Map():
    def __init__(self):
        # TODO to be better when Definitions package available.
        rospy.init_node('map', log_level=rospy.DEBUG)
        self.MapDict = MapLoader().load("../Definitions/map_2018.yml")

        # Starting and publishing the table STL to RViz
        self.markers = MarkersPublisher()
        self.markers.publishTable(self.MapDict)

        # Starting the Get and Set service handlers
        '''
        getter = GetServiceHandler(MapDict)
        setter = SetServiceHandler(MapDict)
        '''
        self.run()

    def run(self):
        r = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.markers.publishZones(self.MapDict)
            self.markers.publishObjects(self.MapDict)
            r.sleep()


if __name__ == "__main__":
    Map()
