#!/usr/bin/python
import rospy
from MapManager import MapLoader, GetServiceHandler, SetServiceHandler
from Markers import MarkersPublisher


class Map():
    def __init__(self):
        self.NODE = rospy.init_node('map', log_level=rospy.DEBUG)
        MapDict = MapLoader().load("../Definitions/map_2018.yml") # TODO to be better when Definitions package available.

		# Starting and publishing the table STL to RViz
        markers = MarkersPublisher()
        markers.publishTable(MapDict)

		# Starting the Get and Set service handlers
		getter = GetServiceHandler(MapDict)
		setter = SetServiceHandler(MapDict)


        r = rospy.Rate(30)
        while not rospy.is_shutdown():
            markers.publishObjects(MapDict)
            r.sleep()


if __name__ == "__main__":
    Map()
