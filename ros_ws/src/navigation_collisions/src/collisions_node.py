#!/usr/bin/python
import rospy
from collisions_subscriptions import CollisionsSubscriptions
from collisions_engine import CollisionLevel, RectObstacle, CircleObstacle, Position
from obstacles_stack import Map, ObstaclesStack
from markers_publisher import MarkersPublisher

from geometry_msgs.msg import Pose2D
from navigation_collisions.msg import PredictedCollision
from navigation_collisions.srv import ActivateCollisions, ActivateCollisionsResponse

class CollisionsNode():
    def __init__(self):
        rospy.init_node("collisions", log_level=rospy.DEBUG)
        self.active = False # navigation/navigator activates this node through a service.

        self.subscriptions = CollisionsSubscriptions()
        Map.Robot = self.subscriptions.create_robot()

        rospy.Service("/navigation/collisions/set_active", ActivateCollisions, self.on_set_active)
        self.pub = rospy.Publisher("/navigation/collisions/warner", PredictedCollision, queue_size=10)

        self.markers = MarkersPublisher()
        self.subscriptions.send_init()
        rospy.loginfo("navigation/collisions ready, waiting for activation.")

        status_services = self._get_status_services("navigation", "collisions")
        status_services.ready(True) # Tell ai/game_status the node initialized successfuly.

        self.run()

    def _get_status_services(self, ns, node_name, arm_cb=None, status_cb=None):
        import sys, os
        sys.path.append(os.environ['UTCOUPE_WORKSPACE'] + '/ros_ws/src/ai_game_status/')
        from init_service import StatusServices
        return StatusServices(ns, node_name, arm_cb, status_cb)

    def run(self):
        r = rospy.Rate(20)
        while not rospy.is_shutdown():
            #ObstaclesStack.updateBeltPoints([RectObstacle(Position(1.5, 0.5, 0.2), 0.3, 0.15)])
            self.subscriptions.update_robot()
            if self.active:
                for c in Map.Robot.check_collisions(ObstaclesStack.toList()):
                    self.publish_collision(c)

            self.markers.publishCheckZones(Map.Robot)
            self.markers.publishObstacles(ObstaclesStack.toList())

            ObstaclesStack.garbageCollect()

            r.sleep()

    def publish_collision(self, collision):
        m = PredictedCollision()
        m.danger_level = collision.level

        if collision.level == CollisionLevel.LEVEL_STOP:
            rospy.logwarn("[COLLISION] Found freaking close collision, please stop !!")
        elif collision.level == CollisionLevel.LEVEL_DANGER:
            rospy.logwarn("[COLLISION] Found close collision intersecting with the path.")
        elif collision.level == CollisionLevel.LEVEL_POTENTIAL:
            rospy.loginfo("[COLLISION] Found far-off collision intersecting with the path.")

        obs = collision.obstacle
        m.obstacle_pos = Pose2D(obs.position.x, obs.position.y, obs.position.a)
        if isinstance(obs, RectObstacle):
            m.obstacle_type = m.TYPE_RECT
            m.obstacle_width, m.obstacle_height = obs.width, obs.height
        elif isinstance(obs, CircleObstacle):
            m.obstacle_type = m.TYPE_CIRCLE
            m.obstacle_radius = obs.radius

        self.pub.publish(m)

    def on_set_active(self, msg):
        self.active = msg.active
        rospy.loginfo("{} collisions check.{}".format("Starting" if self.active else "Stopping", ".." if self.active else ""))
        return ActivateCollisionsResponse(True)

if __name__ == "__main__":
    CollisionsNode()
