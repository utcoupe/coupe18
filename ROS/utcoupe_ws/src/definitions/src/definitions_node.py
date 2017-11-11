#!/usr/bin/env python

from definitions.srv import *
import rospy
import os

EXTENSIONS = ["xml", "yml"]


def callback(req):
    curr_dir = os.path.dirname(os.path.abspath(__file__))
    def_dir = os.path.join(curr_dir, "definitions")

    for ext in EXTENSIONS:
        path = os.path.join(def_dir, "{}/{}.{}"
                            .format(req.domain, req.name.title(), ext))
        if(os.path.isfile(path)):
            return GetDefinitionResponse(path, True)

    #  no file found with any of the extensions
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
