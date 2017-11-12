#!/usr/bin/python
import rospy

class CollisionChecker():
    def __init__(self, robot):
        self.Robot = robot

    def checkGlobal(self, data):
        self.checkStatic(data.CurrentPath, data.StaticObjects)
        self.checkDynamic(data.CurrentPath, data.DynamicObjects)

    def checkStatic(self, path, static_objects):
        rospy.logdebug("Checking collisions with static objects...")

    def checkDynamic(self, path, dynamic_objects):
        rospy.logdebug("Checking collisions with dynamic objects...")
