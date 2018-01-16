#!/usr/bin/python
import rospy
from collisions_subscriptions import CollisionsSubscriptions
from obstacles_stack import ObstaclesStack

from navigation_collisions.srv import ActivateCollisions, ActivateCollisionsResponse

class CollisionsNode():
    def __init__(self):
        rospy.init_node("collisions", log_level=rospy.DEBUG)
        self.active = False # navigation/navigator activates this node through a service.
        self.robot = self.create_robot()

        self.subscriptions = CollisionsSubscriptions()
        rospy.Service("/navigation/collisions/set_active", ActivateCollisions, self.on_set_active)

        self.run()

    def run(self):
        r = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self.active:
                r.sleep()
            ObstaclesStack.garbageCollect()

    def create_robot(self):
        pass

    def on_set_active(self, msg):
        self.active = msg.active
        return ActivateCollisionsResponse(True)

if __name__ == "__main__":
    CollisionsNode()
