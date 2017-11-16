#!/usr/bin/python
import rospy
import tf2_ros

from CollisionsChecker import Robot

from memory_map.msg import MapGet

from processing_belt_interpreter.msg import BeltFiltered
from navigation_navigator.msg import NavStatus
from drivers_asser.msg import RobotSpeed

from geometry_msgs.msg import Pose2D
from navigation_collisions.msg import PredictedCollision


class Data():
    Enemies = []
    BeltPoints = []
    LidarObjects = []

    @staticmethod
    def toList():
        return Data.Enemies + Data.BeltPoints + Data.LidarObjects


class CollisionsNode(object):
    def __init__(self):
        rospy.init_node("collisions", log_level=rospy.DEBUG)
        rospy.loginfo("Started node 'navigation/collisions'.")

        # Preparing to get the robot's position through tf2
        self.tf2_pos_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(5.0))
        self.tf2_pos_listener = tf2_ros.TransformListener(self.tf2_pos_buffer)

        # Subscribing to the dependencies
        rospy.Subscriber("/navigation/navigator/status", NavStatus, self.on_nav_status)
        rospy.Subscriber("/processing/belt_interpreter/points_filtered", BeltFiltered, self.on_belt)
        rospy.Subscriber("/recognition/enemy_tracker/enemies", Enemy, self.on_enemy)
        rospy.Subscriber("/drivers/ard_asserv/robot_speed", RobotSpeed, self.on_robot_speed)

        # Creating the publisher where the collisions will be notified in
        self.pub = rospy.Publisher("/navigation/collisions/", PredictedCollision, queue_size=10)

        # Getting the robot shape and creating the robot instance
        map_get_client = rospy.ServiceProxy("/memory/map/get", MapGet)
        map_get_client.wait_for_service()
        self.Robot = Robot(map_get_client("/entities/{}/shape/*".format(rospy.get_param("/robotname"))))

        self.run()

    def run(self):
        r = rospy.Rate(50)
        while not rospy.is_shutdown():
            try:
                self.Robot.Position = self.tf2_pos_buffer.lookup_transform("robot", "map", rospy.Time())
            except Exception as e:
                rospy.logwarn("Collisions could not get the robot's transform : {}".format(str(e)))

            predicted_collisions = self.Robot.collisions.checkCollisions(Data.toList())
            for pd in predicted_collisions:
                self.publishCollision(pd)

            r.sleep()

    def publishCollision(self, collision): # TODO
        m = PredictedCollision()
        m.danger_level = collision.danger_level
        self.pub.publish(m)

    def on_nav_status(self, msg):
        self.Robot.NavStatus = msg.status # TODO

    def on_belt(self, msg): # TODO
        points_frame = msg.frame_id
        Data.BeltPoints = [StaticObject(p) for p in msg.static_points + msg.dynamic_points]

    def on_lidar_points(self, msg):
        Data.LidarObjects = [] # TODO

    def on_enemy(self, msg): # TODO
        pass

    def on_robot_speed(self, msg):
        self.Robot.Speed.Linear = msg.linear_speed
        self.Robot.Speed.Angular = msg.wheel_speed_right - msg.wheel_speed_left # TODO



if __name__ == "__main__":
    CollisionsNode()
