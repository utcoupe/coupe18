#!/usr/bin/python
import rospy
from collisions_classes import NavStatus, CollisionLevel, Collision

class CollisionChecker():
    def __init__(self, robot):
        self.STOP_DISTANCE   = 0.4 #m
        self.DANGER_DISTANCE = 2.0 #m
        self.Robot = robot

    def check(self, robot, obstacles):
        collisions = []

        if robot.NavMode != NavStatus.STOPPED:
            rospy.logdebug("Checking collisions...")

            path_shapes = robot.CurrentPath.toShapes()

            for path_shape in path_shapes:
                for obstacle in obstacles:
                    # Check if obstacle intersects with one of the path shapes
                    if path_shape.intersects(obstacle):
                        collision_distance = robot.CurrentPath.distanceToCollision(path_shape, obstacle)

                        if collision_distance <= self.STOP_DISTANCE:
                            rospy.logdebug("Found freaking close collision, please stop!!")
                            collisions.append(Collision(obstacle, CollisionLevel.LEVEL_STOP))
                        elif collision_distance <= self.DANGER_DISTANCE:
                            rospy.logdebug("Found dangerous collision.")
                            collisions.append(Collision(obstacle, CollisionLevel.LEVEL_DANGER))

        return collisions
