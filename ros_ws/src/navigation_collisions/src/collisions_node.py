#!/usr/bin/python
import json
import rospy
import tf2_ros

from collisions_checker import *

from memory_map.srv import MapGet

# from processing_belt_interpreter.msg import BeltFiltered
# from navigation_navigator.msg import NavStatus
# from drivers_asser.msg import RobotSpeed

# from geometry_msgs.msg import Pose2D
from navigation_collisions.msg import PredictedCollision
from geometry_msgs.msg import Pose2D


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
        try:
            map_get_client = rospy.ServiceProxy("/memory/map/get", MapGet)
            map_get_client.wait_for_service()
            shape = json.loads(map_get_client("/entities/GR/shape/*").response) # TODO GR must not appear
            if shape["type"] == "rect":
                shape = Rect(shape["width"], shape["height"])
            elif shape["type"] == "circle":
                shape = Circle(shape["radius"])
            else:
                raise ValueError("Robot shape type not supported here.")
            print "got map!"
        except Exception as e:
            rospy.logerr("ERROR Collisions couldn't get the robot's shape from map : " + str(e))
            shape = Rect(0.3, 0.3)
        print shape

        # TESTS
        self.Robot = MapRobot(Circle(0.28)) # Can create a rect or circle
        self.Robot.updatePath(RobotPath([(0.35, 0.75), (0.4, 0.7), (0.8, 1.6), (1, 0.78)]))
        self.Robot.NavStatus = RobotStatus.NAV_STRAIGHT

        MapObstacles.BeltPoints = [MapObstacle(Rect(3.6, 1.4),  Position(0.2, 0.1)),
                                   MapObstacle(Rect(0.24, 0.5), Position(0.6, 0.18)),
                                   MapObstacle(Rect(1.2, 1.98), Position(42, 0.12))]

        self.run()

    def run(self):
        r = rospy.Rate(50)
        while not rospy.is_shutdown():
            self.updateRobotPosition()

            predicted_collisions = self.Robot.checkPathCollisions(MapObstacles.toList())
            for pd in predicted_collisions:
                # print pd.Distance, pd.Level
                self.publishCollision(pd)

            r.sleep()

    def publishCollision(self, collision): # TODO
        m = PredictedCollision()
        m.danger_level = collision.Level

        obs = collision.Obstacle
        m.obstacle_pos = Pose2D(obs.Position.X, obs.Position.Y, obs.Position.A)
        if isinstance(obs, Rect):
            m.obstacle_type = m.TYPE_RECT
            m.width, m.height = obs.Shape.Width, obs.Shape.Height
        elif isinstance(obs, Circle):
            m.obstacle_type = m.TYPE_CIRCLE
            m.radius = obs.Shape.Radius
        # elif isinstance(obs, Point):
        #     m.obstacle_type = m.TYPE_POINT

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
