import math, json
import rospy
import tf2_ros, tf

from obstacles_stack import ObstaclesStack
from collisions_robot import Robot
from engine_shapes import Point, Position, RectObstacle, CircleObstacle
from status_services import StatusServices

from geometry_msgs.msg import PointStamped

from memory_map.srv import MapGet
from navigation_navigator.msg import Status
from processing_belt_interpreter.msg import BeltFiltered

class CollisionsSubscriptions(object):
    def __init__(self):
        # Preparing to get the robot's position, belt frame_id transform.
        self._tf2_pos_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(5.0))
        self._tf2_pos_listener = tf2_ros.TransformListener(self._tf2_pos_buffer)
        self._tranform_listener = tf.TransformListener()

        # Callback buffers
        self._nav_status = None
        self._robot_path_waypoints = None

        # Subscribing to dependencies
        rospy.Subscriber("/navigation/navigator/status", Status, self._on_nav_status)
        rospy.Subscriber("/processing/belt_interpreter/rects_filtered", BeltFiltered, self._on_belt)

        self.game_status = StatusServices("navigation", "collisions", None, self._on_game_status)

    def send_init(self, success = True):
        self.game_status.ready(success)

    def create_robot(self):
        try: # Getting the robot shape and creating the robot instance
            map_get_client = rospy.ServiceProxy("/memory/map/get", MapGet)
            map_get_client.wait_for_service(2.0)
            shape = json.loads(map_get_client("/entities/gr/shape/*").response)
            if not shape["type"] == "rect":
                raise ValueError("Robot shape type not supported here.")
        except Exception as e:
            rospy.logerr("ERROR Collisions couldn't get the robot's shape from map : " + str(e))
            shape = {"width": 0.3, "height": 0.2}
        return Robot(shape["width"], shape["height"]) # Can create a rect or circle

    def update_robot(self, robot):
        new_pos = self._update_robot_pos()
        if new_pos is not None:
            robot.update_position(new_pos)
        if self._nav_status is not None:
            robot.update_status(self._nav_status)
        if self._robot_path_waypoints is not None and len(self._robot_path_waypoints) > 0:
            robot.update_path(self._robot_path_waypoints)

    def _update_robot_pos(self):
        try:
            t = self._tf2_pos_buffer.lookup_transform("map", "robot", rospy.Time())
            tx, ty = t.transform.translation.x, t.transform.translation.y
            rz = self._quaternion_to_euler_angle(t.transform.rotation)[2]
            return (tx, ty, rz)
        except Exception as e:
            return None

    def _on_game_status(self, msg):
        pass

    def _on_nav_status(self, msg):
        self._nav_status = msg.status
        self._robot_path_waypoints = [Point(point.x, point.y) for point in msg.currentPath]

    def _on_belt(self, msg):
        new_belt = []
        for rect in msg.unknown_rects:
            transform = self._tf2_pos_buffer.lookup_transform("map", rect.header.frame_id, # dest frame, source frame
                                                              rospy.Time.now(),            # get the tf at first available time
                                                              rospy.Duration(5.0))         # timeout
            center = PointStamped()
            center.point.x = rect.x
            center.point.y = rect.y
            center.header = rect.header
            try:
                center_map = self._tranform_listener.transformPoint("map", center)
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