#!/usr/bin/python
import math
import rospy
import tf2_ros
import tf2_geometry_msgs

from navigation_navigator.msg import Status
from drivers_ard_asserv.msg import RobotSpeed
# from enemy_tracker import TrackedEnemy
from processing_belt_interpreter.msg import BeltFiltered

from collisions_checker import Map, RectObstacle, RobotStatus, Point, Position, Velocity


class CollisionsSubscriptions(object):
    def __init__(self):
        # Preparing to get the robot's position, belt frame_id transform.
        self.tf2_pos_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(5.0))
        self.tf2_pos_listener = tf2_ros.TransformListener(self.tf2_pos_buffer)

        # Subscribing to the dependencies
        rospy.Subscriber("/navigation/navigator/status", Status, self.on_nav_status)
        rospy.Subscriber("/processing/belt_interpreter/points_filtered", BeltFiltered, self.on_belt)
        # rospy.Subscriber("/recognition/enemy_tracker/enemies", TrackedEnemy, self.on_enemy)
        rospy.Subscriber("/drivers/ard_asserv/robot_speed", RobotSpeed, self.on_robot_speed)

    def updateRobotPosition(self):
        try:
            t = self.tf2_pos_buffer.lookup_transform("map", "robot", rospy.Time())
            tx, ty = t.transform.translation.x, t.transform.translation.y
            rz = self._quaternion_to_euler_angle(t.transform.rotation)[2]
            Map.Robot.updatePosition(Position(tx, ty, angle = rz))
        except Exception as e:
            rospy.logwarn("Collisions could not get the robot's pos transform : {}".format(str(e)))

    def on_nav_status(self, msg):
        Map.Robot.NavStatus = msg.status
        Map.Robot.updatePath([Point(point.x, point.y) for point in msg.currentPath])

    def on_belt(self, msg):
        Map.BeltPoints = []
        for rect in msg.map_points + msg.unknown_points:
            transform = self.tf2_pos_buffer.lookup_transform("/map", msg.frame_id, # dest frame, source frame
                                                             rospy.Time.now(),     # get the tf at first available time
                                                             rospy.Duration(1.0))  # timeout
            map_pose = tf2_geometry_msgs.do_transform_pose(msg.frame_id, transform)
            Map.BeltPoints.append(RectObstacle(Position(map_pose.pose.position.x, map_pose.pose.position.y,
                                                        self._quaternion_to_euler_angle(map_pose.pose.orientation)),
                                               rect.w, rect.h, velocity = Velocity(0, 0)))

    def on_lidar_points(self, msg):
        Map.LidarObjects = [] # TODO

    def on_enemy(self, msg): # TODO
        pass

    def on_robot_speed(self, msg):
        Map.Robot.Velocity.Linear = msg.linear_speed
        Map.Robot.Velocity.Angular = 0.0 # TODO Implement here if we use it one day ?

    def _quaternion_to_euler_angle(self, quaternion):
        # https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        w, x, y, z = quaternion.w, quaternion.x, quaternion.y, quaternion.z
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y ** 2)
        X = math.degrees(math.atan2(t0, t1))

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        Y = math.degrees(math.asin(t2))

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y ** 2 + z * z)
        Z = math.atan2(t3, t4)
        return X, Y, Z