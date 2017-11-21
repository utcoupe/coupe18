#!/usr/bin/python
import json, time
import rospy
import tf2_ros

from collisions import *

# from memory_map.msg import MapGet

# from processing_belt_interpreter.msg import BeltFiltered
# from navigation_navigator.msg import NavStatus
# from drivers_asser.msg import RobotSpeed

# from geometry_msgs.msg import Pose2D
from navigation_collisions.msg import PredictedCollision


class CollisionsNode(object):
    def __init__(self):
        rospy.init_node("collisions", log_level=rospy.DEBUG)
        rospy.loginfo("Started node 'navigation/collisions'.")

        # Preparing to get the robot's position through tf2
        self.tf2_pos_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(5.0))
        self.tf2_pos_listener = tf2_ros.TransformListener(self.tf2_pos_buffer)

        # Subscribing to the dependencies
        # rospy.Subscriber("/navigation/navigator/status", NavStatus, self.on_nav_status)
        # rospy.Subscriber("/processing/belt_interpreter/points_filtered", BeltFiltered, self.on_belt)
        # rospy.Subscriber("/recognition/enemy_tracker/enemies", Enemy, self.on_enemy)
        # rospy.Subscriber("/drivers/ard_asserv/robot_speed", RobotSpeed, self.on_robot_speed)

        # Creating the publisher where the collisions will be notified in
        self.pub = rospy.Publisher("/navigation/collisions/warner", PredictedCollision, queue_size=10)

        # Getting the robot shape and creating the robot instance
        # try:
        #     map_get_client = rospy.ServiceProxy("/memory/map/get", MapGet)
        #     map_get_client.wait_for_service()
        #     shape = Shape(json.loads(map_get_client("/entities/{}/shape/*".format(rospy.get_param("/robotname")))))
        # except:
        #     shape = Shape({"type": "rect", "width": 0.3, "height": 0.3})
        # self.Robot = MapRobot(shape)

        self.Robot = MapRobot(Rect(0.42, 0.28)) # TODO handle circles
        self.Robot.updatePath(RobotPath([(0.35, 0.75), (0.4, 0.7), (0.8, 1.6)]))
        self.Robot.NavStatus = RobotStatus.NAV_STRAIGHT

        MapObstacles.BeltPoints = [(3.6, 1.4), (0.75, 0.5)]

        time.sleep(0.2)  # wait for tfs to arrive
        self.run()

    def run(self):
        r = rospy.Rate(50)
        while not rospy.is_shutdown():
            self.updateRobotPosition()

            predicted_collisions = self.Robot.checkPathCollisions(MapObstacles.toList())
            for pd in predicted_collisions:
                print pd.Distance, pd.Level
                # self.publishCollision(pd)

            r.sleep()

    def publishCollision(self, collision): # TODO
        m = PredictedCollision()
        m.danger_level = collision.danger_level
        self.pub.publish(m)

    def updateRobotPosition(self):
        try:
            t = self.tf2_pos_buffer.lookup_transform("robot", "map", rospy.Time())
            tx, ty = t.transform.translation.x, t.transform.translation.y
            rz = t.transform.rotation.z
            self.Robot.updatePosition(Position(tx, ty, angle = rz))
        except Exception as e:
            rospy.logwarn("Collisions could not get the robot's pos transform : {}".format(str(e)))

    def on_nav_status(self, msg):
        if msg.status == msg.NAV_STOPPED:
            self.Robot.NavStatus = RobotStatus.NAV_STOPPED
        elif msg.status == msg.NAV_STRAIGHT:
            self.Robot.NavStatus = RobotStatus.NAV_STRAIGHT
        elif msg.status == msg.NAV_TURNING:
            self.Robot.NavStatus = RobotStatus.NAV_TURNING
        else:
            rospy.logerr("ERROR : Unrecognized robot status type.")

        self.Robot.updatePath(msg.path) # TODO

    def on_belt(self, msg): # TODO
        points_frame = msg.frame_id
        MapObstacles.BeltPoints = [MapObstacle(p) for p in msg.static_points + msg.dynamic_points]

    def on_lidar_points(self, msg):
        MapObstacles.LidarObjects = [] # TODO

    def on_enemy(self, msg): # TODO
        pass

    def on_robot_speed(self, msg):
        self.Robot.Velocity.Linear = msg.linear_speed
        self.Robot.Velocity.Angular = msg.wheel_speed_right - msg.wheel_speed_left # TODO



if __name__ == "__main__":
    CollisionsNode()
