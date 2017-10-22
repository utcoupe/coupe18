#!/usr/bin/python
import rospy
import memory_map.srv
'''
SetServiceHandler : Takes care of returning the right elements based on the service request call.
'''


class SetServiceHandler():
    def __init__(self, mapdict):
        self.SetSERV = rospy.Service("get", memory_map.srv.MapSet, self.serv_handler_on_map_get_request)
        self.MapDict = mapdict

    def serv_handler_on_map_get_request(self, req):
        pass
