#!/usr/bin/python
import rospy
import tf2_ros

from CollisionsChecker import *

from geometry_msgs.msg import Pose2D
# from memory_map.msg import MapGet
# from navigation_navigator.msg import NavStatus
# from processing_belt_interpreter.msg import BeltFiltered
from drivers_asser.msg import RobotSpeed
from navigation_collisions.msg import PredictedCollision

class Data():
    Robot = None
    BeltPoints = []
    LidarObjects = []


class CollisionsNode(object):
    def __init__(self):
        rospy.init_node("collisions", log_level=rospy.DEBUG)
        rospy.loginfo("Started node 'navigation/collisions'.")

        # Preparing to get the robot's position through tf2
        self.tf2_pos_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(5.0))
        self.tf2_pos_listener = tf2_ros.TransformListener(self.tf2_pos_buffer)

        # Subscribing to the dependencies
        # rospy.Subscriber("/navigation/navigator/status", NavStatus, self.on_nav_status)
        # rospy.Subscriber("/processing/belt_interpreter/points", BeltFiltered, self.on_belt)
        # rospy.Subscriber("/drivers/ard_asserv/robot_speed", RobotSpeed, self.on_robot_speed)

        # Creating the publisher where the collisions will be notified in
        self.pub = rospy.Publisher("/navigation/collisions/", PredictedCollision, queue_size=10)

        # Getting the robot shape and creating the robot instance
        # map_get_client = rospy.ServiceProxy("/memory/map/get", MapGet)
        # map_get_client.wait_for_service(3)
        # Data.Robot = Robot(map_get_client("/entities/{}/shape/*".format(rospy.get_param("/robotname"))))

        self.checker = CollisionChecker(Data.Robot)

        self.run()

    def run(self):
        r = rospy.Rate(50)
        while not rospy.is_shutdown():
            try:
                Data.Robot.Position = self.tf2_pos_buffer.lookup_transform("robot", "map", rospy.Time())
            except Exception as e:
                rospy.logwarn("Collisions could not get the robot's transform : {}".format(str(e)))

            if Data.Robot.Navigating:
                self.checker.checkGlobal(Data)
            r.sleep()

    def on_nav_status(self, msg):
        Data.Robot.Navigating = msg.status # TODO

    def on_belt(self, msg):
        points_frame = msg.frame_id
        Data.BeltPoints.TerrainPoints  = [StaticObject(p) for p in msg.static_points]
        Data.BeltPoints.ObstaclePoints = [StaticObject(p) for p in msg.dynamic_points]

    def on_robot_speed(self, msg):
        Data.Robot.Speed.Linear = msg.linear_speed
        Data.Robot.Speed.Linear = msg.wheel_speed_right - msg.wheel_speed_left



if __name__ == "__main__":
    CollisionsNode()
