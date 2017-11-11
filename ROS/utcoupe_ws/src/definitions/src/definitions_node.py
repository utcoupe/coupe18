#!/usr/bin/env python

from definitions.srv import *
import rospy
import os


def callback(req):
    curr_dir = os.path.dirname(os.path.abspath(__file__))
    def_dir = os.path.join(curr_dir, "definitions")
    req_split = req.request.split('/')
    
    if req_split[0] in ["rviz", "map"]:
        req_final = req.request
    else
        req_final = "robots/{}/{}".format(rospy.get_param("/robot").lower(), req.request)


    path = os.path.join(def_dir, req_final)
    if(os.path.isfile(path)):
        return GetDefinitionResponse(path, True)

    else:
        rospy.logerr("[MEMORY] Request failed, file {} does not exist !"
                     .format(path))
        return GetDefinitionResponse("", False)


def server():
    rospy.init_node('definitions')
    s = rospy.Service('memory/definitions', GetDefinition, callback)
    print "[MEMORY] Definitions server ready"
    rospy.spin()


if __name__ == "__main__":
    server()
