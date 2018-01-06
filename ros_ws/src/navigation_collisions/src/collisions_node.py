#!/usr/bin/python
import json
import rospy

from collisions_checker import *
from collisions_subscriptions import CollisionsSubscriptions
from markers_publisher import MarkersPublisher

from memory_map.srv import MapGet

from navigation_collisions.msg import PredictedCollision
from geometry_msgs.msg import Pose2D


class CollisionsNode(object):
    def __init__(self):
        rospy.init_node("collisions", log_level=rospy.INFO)

        # Creating listeners
        self.subscriptions = CollisionsSubscriptions()

        # Creating the publisher where the collisions will be notified in
        self.pub = rospy.Publisher("/navigation/collisions/warner", PredictedCollision, queue_size=10)
        self.markers = MarkersPublisher()

        # Getting the robot shape and creating the robot instance
        try:
            map_get_client = rospy.ServiceProxy("/memory/map/get", MapGet)
            map_get_client.wait_for_service(2.0)
            shape = json.loads(map_get_client("/entities/GR/shape/*").response) # TODO GR must not appear
            if not shape["type"] == "rect":
                raise ValueError("Robot shape type not supported here.")
        except Exception as e:
            rospy.logerr("ERROR Collisions couldn't get the robot's shape from map : " + str(e))
            shape = {"width": 0.3, "height": 0.2}
        Map.Robot = MapRobot(shape["width"], shape["height"]) # Can create a rect or circle

        rospy.loginfo("'navigation/collisions' initialized.")

        # TESTS While dependencies not available.
        # Map.Robot.updatePath(RobotPath([Point(2.0, 1.7), Point(0.6, 0.7), Point(2.2, 0.6), Point(2.7, 0.3)]))
        # Map.Robot.NavStatus = RobotStatus.NAV_NAVIGATING
        # Map.Robot.updateVelocity(0.2, 0.33)

        # Map.BeltPoints = [RectObstacle(Position(0.25, 1.3, 0.78539816339), 0.1,  0.2),
        #                   RectObstacle(Position(1.6, 0.9, 3.14 / 2),       0.24, 0.5),
        #                   CircleObstacle(Position(2.15, 0.7),              0.25 / 2)]

    def run(self):
        r = rospy.Rate(15)
        while not rospy.is_shutdown():
            self.subscriptions.updateRobotPosition()

            predicted_collisions = Map.Robot.checkCollisions(Map.toList())
            for pd in predicted_collisions:
                self.publishCollision(pd)

            self.markers.publishCheckZones(Map.Robot)
            self.markers.publishObstacles(Map.toList())

            r.sleep()

    def publishCollision(self, collision):
        m = PredictedCollision()
        m.danger_level = collision.Level

        if collision.Level == CollisionLevel.LEVEL_STOP:
            rospy.logwarn("[COLLISIONS] Found freaking close collision, please stop !!")
        elif collision.Level == CollisionLevel.LEVEL_DANGER:
            rospy.loginfo("[COLLISIONS] Found collision intersecting with the path.")

        obs = collision.Obstacle
        m.obstacle_pos = Pose2D(obs.Position.X, obs.Position.Y, obs.Position.A)
        if isinstance(obs, RectObstacle):
            m.obstacle_type = m.TYPE_RECT
            m.obstacle_width, m.obstacle_height = obs.Width, obs.Height
        elif isinstance(obs, CircleObstacle):
            m.obstacle_type = m.TYPE_CIRCLE
            m.obstacle_radius = obs.Radius

        self.pub.publish(m)

# Entry point
if __name__ == "__main__":
    c = CollisionsNode()
    c.run()
