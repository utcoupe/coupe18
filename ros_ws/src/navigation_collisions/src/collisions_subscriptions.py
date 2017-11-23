#!/usr/bin/python
import rospy
from processing_belt_interpreter.msg import BeltFiltered
# from navigation_navigator.msg import NavStatus
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
        if msg.status == msg.NAV_STOPPED:
            Map.Robot.NavStatus = RobotStatus.NAV_STOPPED
        elif msg.status == msg.NAV_STRAIGHT:
            Map.Robot.NavStatus = RobotStatus.NAV_STRAIGHT
        elif msg.status == msg.NAV_TURNING:
            Map.Robot.NavStatus = RobotStatus.NAV_TURNING
        else:
            rospy.logerr("ERROR : Unrecognized robot status type.")

        Map.Robot.updatePath(msg.path) # TODO

    def on_belt(self, msg): # TODO
        points_frame = msg.frame_id
        Map.BeltPoints = [MapObstacle(None, None) for p in msg.static_points + msg.dynamic_points]

    def on_lidar_points(self, msg):
        Map.LidarObjects = [] # TODO

    def on_enemy(self, msg): # TODO
        pass

    def on_robot_speed(self, msg):
        Map.Robot.Velocity.Linear = msg.linear_speed
        Map.Robot.Velocity.Angular = msg.wheel_speed_right - msg.wheel_speed_left # TODO
