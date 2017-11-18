#!/usr/bin/env python
import roslib
from ai_scheduler.srv import *
import rospy

def handle(req):
    print "Returning {}".format(req)
    return True

def server():
    rospy.init_node('server_test')
    s = rospy.Service("movement/navigator/goto", test, handle)
    print "Ready."
    rospy.spin()

if __name__ == "__main__":
    server()
