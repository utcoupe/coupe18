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
            shape = Rect(0.3, 0.2)

        # TESTS While dependencies not available.
        Map.Robot = MapRobot(shape) # Can create a rect or circle
        Map.Robot.updatePath(RobotPath([Point(2.0, 1.7), Point(0.6, 0.7), Point(2.2, 0.6), Point(2.7, 0.3)]))
        Map.Robot.NavStatus = RobotStatus.NAV_STRAIGHT

        Map.BeltPoints = [RectObstacle(Rect(0.1, 0.2),  Position(0.25, 1.3, 0.78539816339)),
                          RectObstacle(Rect(0.24, 0.5), Position(1.6, 0.9, 3.14 / 2)),
                          RectObstacle(Circle(0.25 / 2), Position(2.55, 0.55))]

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
            self.markers.publishObstacles(Map.toList())

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
            t = self.tf2_pos_buffer.lookup_transform("map", "robot", rospy.Time())
            tx, ty = t.transform.translation.x, t.transform.translation.y
            rz = t.transform.rotation.z
            Map.Robot.updatePosition(Position(tx, ty, angle = rz))
        except Exception as e:
            rospy.logwarn("Collisions could not get the robot's pos transform : {}".format(str(e)))

# Entry point
if __name__ == "__main__":
    CollisionsNode()
