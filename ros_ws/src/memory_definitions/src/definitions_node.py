#!/usr/bin/env python

from memory_definitions.srv import *
from ai_game_status import StatusServices
import rospy
import os


def callback(req):
    curr_dir = os.path.dirname(os.path.abspath(__file__))
    def_dir = os.path.join(curr_dir, "../def")
    req_split = req.request.split('/')

    if req_split[0] in ["ai", "memory", "navigation", "movement", "processing", "recognition", "drivers"]:
        if not rospy.has_param('/robot'):
            rospy.logerr("Parameter '/robot' not set, cannot provide the definition file {}".format(req.request))
            req_final = ""
        else:
            req_final = "robots/{}/{}".format(rospy.get_param("/robot").lower(), req.request)
            if req_split[0] == "ai" and req_split[1] == "system_orders.xml": # exception.
                req_final = req.request
    else:
        req_final = req.request


    path = os.path.join(def_dir, req_final)
    if os.path.isfile(path):
        return GetDefinitionResponse(path, True)
    else:
        rospy.logerr("Request failed, file {} does not exist !".format(path))
        return GetDefinitionResponse("", False)


def server():
    rospy.init_node('definitions')
    s = rospy.Service('/memory/definitions/get', GetDefinition, callback)
    rospy.logdebug("Definitions server ready")

    # Tell ai/game_status the node initialized successfuly.
    StatusServices("memory", "definitions").ready(True)

    rospy.spin()


if __name__ == "__main__":
    server()
