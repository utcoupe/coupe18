#!/usr/bin/python
import rospy


class CollisionLevel(object):
    LEVEL_STOP = 0
    LEVEL_DANGER = 1
    LEVEL_POTENTIAL = 2


class Collision(object):
    def __init__(self, collision_level, obstacle, travel_distance):
        self.Distance = travel_distance
        self.Obstacle = obstacle
        self.Level = collision_level


class PathChecker(object):
    STOP_DISTANCE = 0.4 #m
    DANGER_DISTANCE = 2.0 #m

    def checkCollisions(self, robot, path, obstacles):
        collisions = []

        distance_to_collision = 0.0
        for path_rect in path:
            for obstacle in obstacles:
                # Check if obstacle intersects with one of the path shapes
                if path_rect.intersects(obstacle):
                    travel_distance = path_rect.distanceToCollision(obstacle)

                    if travel_distance <= self.STOP_DISTANCE:
                        rospy.logwarn("[COLLISION] Found freaking close collision, please stop!!")
                        level = CollisionLevel.LEVEL_STOP
                    elif travel_distance <= self.DANGER_DISTANCE:
                        rospy.loginfo("[COLLISION] Found dangerous collision.")
                        level = CollisionLevel.LEVEL_DANGER

                    collisions.append(Collision(level, obstacle, travel_distance))
            distance_to_collision += path_rect.Width

        return collisions
