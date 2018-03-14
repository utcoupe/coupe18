import math, json
import rospy
import tf2_ros, tf

from obstacles_stack import Map, ObstaclesStack
from collisions_robot import Robot
from collisions_engine import Point, Position, Velocity, SegmentObstacle, RectObstacle, CircleObstacle
from status_services import StatusServices

from geometry_msgs.msg import PointStamped

from memory_map.srv import MapGet
from navigation_navigator.msg import Status
from drivers_ard_asserv.msg import RobotSpeed
from processing_belt_interpreter.msg import BeltRects
from processing_lidar_objects.msg import Obstacles

class CollisionsSubscriptions(object):
    def __init__(self):
        # Preparing to get the robot's position, belt frame_id transform.
        self._tf2_pos_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(5.0))
        self._tf2_pos_listener = tf2_ros.TransformListener(self._tf2_pos_buffer)
        self._transform_listener = tf.TransformListener()

        # Callback buffers
        self._nav_status = 0 #STATUS_IDLE
        self._robot_path_waypoints = []
        self._vel_linear  = 0.0
        self._vel_angular = 0.0

        # Subscribing to dependencies
        rospy.Subscriber("/navigation/navigator/status", Status, self._on_nav_status)
        rospy.Subscriber("/processing/belt_interpreter/rects", BeltRects, self._on_belt)
        rospy.Subscriber("/processing/lidar_objects/obstacles", Obstacles, self._on_lidar)
        rospy.Subscriber("/drivers/ard_asserv/speed", RobotSpeed, self.on_robot_speed)

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
            shape = {"width": 0.4, "height": 0.25}
        return Robot(shape["width"], shape["height"]) # Can create a rect or circle

    def update_robot(self):
        new_pos = self._update_robot_pos()
        if new_pos is not None:
            Map.Robot.update_position(new_pos)

        if self._nav_status is not None:
            Map.Robot.update_status(self._nav_status)
        if self._robot_path_waypoints is not None and len(self._robot_path_waypoints) > 0:
            Map.Robot.update_waypoints(self._robot_path_waypoints)

        Map.Robot.update_velocity(self._vel_linear, self._vel_angular)

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
        for rect in msg.rects:
            transform = self._tf2_pos_buffer.lookup_transform("map", rect.header.frame_id, # dest frame, source frame
                                                              rospy.Time.now(),            # get the tf at first available time
                                                              rospy.Duration(5.0))         # timeout
            center = PointStamped()
            center.point.x = rect.x
            center.point.y = rect.y
            center.header = rect.header
            try:
                center_map = self._transform_listener.transformPoint("map", center)
                new_belt.append(RectObstacle(Position(center_map.point.x, center_map.point.y,
                                                      self._quaternion_to_euler_angle(transform.transform.rotation)[2]),
                                                      rect.w, rect.h))
            except:
                rospy.logdebug("Frame /map does not exist, cannot fetch belt rects.")
        if len(new_belt) > 0:
            ObstaclesStack.updateBeltPoints(new_belt)

    def _on_lidar(self, msg):
        new_lidar = []
        if msg.header.frame_id != "map":
            rospy.logwarn("Lidar obstacles not in /map tf frame, skipping.")
            return
        for segment in msg.segments:
            new_lidar.append(SegmentObstacle(Position(segment.first_point.x, segment.first_point.y),
                                             Position(segment.last_point.x,  segment.last_point.y)))
        for circle in msg.circles:
            vel_d = math.sqrt(circle.velocity.y ** 2 + circle.velocity.x ** 2)
            vel_a = math.atan2(circle.velocity.y, circle.velocity.x)
            new_lidar.append(CircleObstacle(Position(circle.center.x, circle.center.y, angle = vel_a),
                                            circle.radius, velocity = Velocity(circle.radius * 2, circle.radius * math.sqrt(3.0) / 2.0, 
                                            vel_d, 0.0)))

        if len(new_lidar) > 0:
            ObstaclesStack.updateLidarObjects(new_lidar)


    def on_robot_speed(self, msg):
        self._vel_linear = msg.linear_speed
        self._vel_angular = 0.0

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
