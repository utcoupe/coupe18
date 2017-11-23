#!/usr/bin/python
import json
import rospy
import tf2_ros

from collisions_checker import *
from collisions_subscriptions import CollisionsSubscriptions
from markers_publisher import MarkersPublisher

from memory_map.srv import MapGet

from navigation_collisions.msg import PredictedCollision
from geometry_msgs.msg import Pose2D


class CollisionsNode(object):
    def __init__(self):
        rospy.init_node("collisions", log_level=rospy.DEBUG)
        rospy.loginfo("Started node 'navigation/collisions'.")

        # Preparing to get the robot's position through tf2
        self.tf2_pos_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(5.0))
        self.tf2_pos_listener = tf2_ros.TransformListener(self.tf2_pos_buffer)

        # Creating listeners
        # CollisionsSubscriptions() TODO Resolve dependencies

        # Creating the publisher where the collisions will be notified in
        self.pub = rospy.Publisher("/navigation/collisions/warner", PredictedCollision, queue_size=10)

        self.markers = MarkersPublisher()

        # Getting the robot shape and creating the robot instance
        try:
            map_get_client = rospy.ServiceProxy("/memory/map/get", MapGet)
            map_get_client.wait_for_service(2.0)
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

        # TESTS While dependencies not available.
        Map.Robot = MapRobot(shape) # Can create a rect or circle
        Map.Robot.updatePath(RobotPath([Point(0.8, 0.75), Point(1.4, 0.7), Point(0.8, 1.6), Point(1.8, 0.78)]))
        Map.Robot.NavStatus = RobotStatus.NAV_STRAIGHT

        Map.BeltPoints = [MapObstacle(Rect(3.6, 1.4),  Position(0.2, 0.1)),
                          MapObstacle(Rect(0.24, 0.5), Position(0.6, 0.18)),
                          MapObstacle(Rect(1.2, 1.98), Position(42, 0.12))]

        self.run()

    def run(self):
        r = rospy.Rate(50)
        while not rospy.is_shutdown():
            self.updateRobotPosition()

            predicted_collisions = Map.Robot.checkPathCollisions(Map.toList())
            for pd in predicted_collisions:
                # print pd.Distance, pd.Level
                self.publishCollision(pd)

            self.markers.publishPathShapes(Map.Robot)

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
            Map.Robot.updatePosition(Position(tx, ty, angle = rz))
        except Exception as e:
            rospy.logwarn("Collisions could not get the robot's pos transform : {}".format(str(e)))

# Entry point
if __name__ == "__main__":
    CollisionsNode()
