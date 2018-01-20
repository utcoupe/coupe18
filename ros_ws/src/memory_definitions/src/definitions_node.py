#!/usr/bin/env python

from memory_definitions.srv import *
import rospy
import os


def callback(req):
    curr_dir = os.path.dirname(os.path.abspath(__file__))
    def_dir = os.path.join(curr_dir, "../def")
    req_split = req.request.split('/')

    if req_split[0] in ["ai", "memory", "navigation", "processing", "recognition"]:
        if not rospy.has_param('/robot'):
            rospy.logerr("Parameter '/robot' not set, cannot provide the definition file {}".format(req.request))
            req_final = ""
        else:
            req_final = "robots/{}/{}".format(rospy.get_param("/robot").lower(), req.request)
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

    status_services = _get_status_services("memory", "definitions")
    status_services.ready(True) # Tell ai/game_status the node initialized successfuly.

    rospy.spin()


def _get_status_services(ns, node_name, arm_cb=None, status_cb=None):
    import sys
    sys.path.append(os.environ['UTCOUPE_WORKSPACE'] + '/ros_ws/src/ai_game_status/')
    from init_service import StatusServices
    return StatusServices(ns, node_name, arm_cb, status_cb)


if __name__ == "__main__":
    server()
