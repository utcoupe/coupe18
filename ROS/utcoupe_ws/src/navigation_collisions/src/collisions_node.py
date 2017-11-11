#!/usr/bin/python
import rospy

from collision_checker import CollisionChecker

from geometry_msgs.msg import Pose2D
from navigation_navigator.msg import NavStatus
from processing_belt_interpreter.msg import BeltFiltered
from navigation_collisions.msg import PredictedCollision

class CollisionsNode(object):
    def __init__(self):
        rospy.init_node("collisions", log_level=rospy.DEBUG)
        rospy.loginfo("Started node 'navigation/collisions'.")

        rospy.Subscriber("/drivers/ard_asserv/position", Pose2D, self.on_position)
        rospy.Subscriber("/navigation/navigator/status", NavStatus, self.on_nav_status)
        rospy.Subscriber("/processing/belt_interpreter/points", BeltFiltered, self.on_belt)

        self.pub = rospy.Publisher("/navigation/collisions/", PredictedCollision, queue_size=10)

        self.checker = CollisionChecker()

    def on_position(self, msg):
        pass

    def on_nav_status(self, msg):
        pass

    def on_belt(self, msg):
        pass



if __name__ == "__main__":
    CollisionsNode()
