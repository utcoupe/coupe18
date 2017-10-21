#!/usr/bin/python
import rospy

'''
SetServiceHandler : Takes care of returning the right elements based on the service request call.
'''
class SetServiceHandler():
    def __init__(self, mapdict):
        self.SetSERV = rospy.Service("get", TYPEHERE, self.serv_handler_on_map_get_request)
        self.MapDict = mapdict

    def serv_handler_on_map_get_request(self, req):
        pass
'''
GET REQUESTS STRUCTURE :
    -
'''
