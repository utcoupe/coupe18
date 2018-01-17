#!/usr/bin/python
import rospy
from collisions_subscriptions import CollisionsSubscriptions
from engine_constants import CollisionLevel
from obstacles_stack import ObstaclesStack

from navigation_collisions.msg import PredictedCollision
from navigation_collisions.srv import ActivateCollisions, ActivateCollisionsResponse

class CollisionsNode():
    def __init__(self):
        rospy.init_node("collisions", log_level=rospy.DEBUG)
        self.active = False # navigation/navigator activates this node through a service.

        self.subscriptions = CollisionsSubscriptions()
        self.robot = self.subscriptions.create_robot()
        rospy.Service("/navigation/collisions/set_active", ActivateCollisions, self.on_set_active)

        self.subscriptions.send_init()
        rospy.loginfo("navigation/collisions ready, waiting for activation.")

        self.run()

    def run(self):
        r = rospy.Rate(20)
        while not rospy.is_shutdown():
            self.subscriptions.update_robot(self.robot)
            if self.active:
                collisions = self.robot.check_collisions(ObstaclesStack.toList())
            ObstaclesStack.garbageCollect()

            r.sleep()
            rospy.logerr(len(ObstaclesStack.toList()))

    def publish_collision(self, collision):
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

    def on_set_active(self, msg):
        self.active = msg.active
        rospy.loginfo("{} collisions check.{}".format("Starting" if self.active else "Stopping", ".." if self.active else ""))
        return ActivateCollisionsResponse(True)

if __name__ == "__main__":
    CollisionsNode()
