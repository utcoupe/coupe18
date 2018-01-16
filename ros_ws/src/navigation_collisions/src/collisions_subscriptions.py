import math
import rospy
import tf2_ros, tf

from obstacles_stack import ObstaclesStack
from collisions_shapes import Point, Position, RectObstacle, CircleObstacle

from geometry_msgs.msg import PointStamped

from navigation_navigator.msg import Status
from processing_belt_interpreter.msg import RectsFiltered

class CollisionsSubscriptions(object):
    def __init__(self):
        # Preparing to get the robot's position, belt frame_id transform.
        self.tf2_pos_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(5.0))
        self.tf2_pos_listener = tf2_ros.TransformListener(self.tf2_pos_buffer)
        self.tranform_listener = tf.TransformListener()

        # Subscribing to dependencies
        rospy.Subscriber("/navigation/navigator/status", Status, self.on_nav_status)
        rospy.Subscriber("/processing/belt_interpreter/rects_filtered", RectsFiltered, self.on_belt)

    def on_nav_status(self, msg):
        ObstaclesStack.Robot.NavStatus = msg.status
        ObstaclesStack.Robot.updatePath([Point(point.x, point.y) for point in msg.currentPath])

    def on_belt(self, msg):
        new_belt = []
        for rect in msg.map_rects + msg.unknown_rects:
            transform = self.tf2_pos_buffer.lookup_transform("map", rect.header.frame_id, # dest frame, source frame
                                                             rospy.Time.now(),            # get the tf at first available time
                                                             rospy.Duration(1.0))         # timeout
            center = PointStamped()
            center.point.x = rect.x
            center.point.y = rect.y
            center.header = rect.header
            try:
                center_map = self.tranform_listener.transformPoint("map", center)
                new_belt.append(RectObstacle(Position(center_map.point.x, center_map.point.y,
                                                      self._quaternion_to_euler_angle(transform.transform.rotation)[2]),
                                                      rect.w, rect.h))
            except:
                rospy.logdebug("Frame /map does not exist, cannot fetch belt rects.")
        if len(new_belt) > 0:
            ObstaclesStack.updateBeltPoints(new_belt)

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
        return (X, Y, Z)