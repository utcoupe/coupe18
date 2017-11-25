#!/usr/bin/python
import rospy
from collisions_checker import Point
from processing_belt_interpreter.msg import BeltFiltered
# from navigation_navigator.msg import Status
# from drivers_asser.msg import RobotSpeed

from collisions_checker import Map, MapObstacle, RobotStatus


class CollisionsSubscriptions(object):
    def __init__(self):
        # Subscribing to the dependencies
        rospy.Subscriber("/navigation/navigator/status", NavStatus, self.on_nav_status)
        rospy.Subscriber("/processing/belt_interpreter/points_filtered", BeltFiltered, self.on_belt)
        rospy.Subscriber("/recognition/enemy_tracker/enemies", Enemy, self.on_enemy)
        rospy.Subscriber("/drivers/ard_asserv/robot_speed", RobotSpeed, self.on_robot_speed)

    def on_nav_status(self, msg):
        Map.Robot.NavStatus = msg.status
        for point in msg.currentPath:
            Map.Robot.updatePath(Point(point.x, point.y))

    def on_belt(self, msg): # TODO
        points_frame = msg.frame_id
        Map.BeltPoints = [MapObstacle(None, None) for p in msg.static_points + msg.dynamic_points]

    def on_lidar_points(self, msg):
        Map.LidarObjects = [] # TODO

    def on_enemy(self, msg): # TODO
        pass

    def on_robot_speed(self, msg):
        Map.Robot.Velocity.Linear = msg.linear_speed
        Map.Robot.Velocity.Angular = 0.0 # TODO Implement here if we use it one day ?
