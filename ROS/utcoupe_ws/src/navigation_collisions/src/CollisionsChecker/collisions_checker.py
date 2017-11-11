#!/usr/bin/python
import rospy

class CollisionChecker():
    def __init__(self):
        pass

    def checkGlobal(self, robot, path, static_objects, dynamic_objects):
        self.checkStatic(robot, path, static_objects)
        self.checkDynamic(robot, path, dynamic_objects)

    def checkStatic(self, robot, path, static_objects):
        rospy.logdebug("Checking collisions with static objects...")

    def checkDynamic(self, robot, path, dynamic_objects):
        rospy.logdebug("Checking collisions with dynamic objects...")
