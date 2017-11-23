#!/usr/bin/python
import math
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

    def checkCollisions(self, robot, path_shapes, obstacles):
        collisions = []

        distance_to_collision = 0.0
        for path_segment in path_shapes:
            for obstacle in obstacles:
                # Check if obstacle intersects with one of the path shapes
                if path_segment.intersects(obstacle):
                    travel_distance = path_segment.distanceToCollision(obstacle)

                    if travel_distance <= self.STOP_DISTANCE:
                        rospy.logwarn("[COLLISION] Found freaking close collision, please stop!!")
                        level = CollisionLevel.LEVEL_STOP
                    elif travel_distance <= self.DANGER_DISTANCE:
                        rospy.loginfo("[COLLISION] Found dangerous collision.")
                        level = CollisionLevel.LEVEL_DANGER

                    collisions.append(Collision(level, obstacle, distance_to_collision + travel_distance))
            distance_to_collision += path_segment.Shape.Width
        return collisions

class Intersections():
    

    def collision(rect, circle):
        # rleft, rtop, width, height,   # rectangle definition
        #         center_x, center_y, radius):  # circle definition
        """ Detect collision between a rectangle and circle. """

        # complete boundbox of the rectangle
        rright, rbottom = rleft + width/2, rtop + height/2

        # bounding box of the circle
        cleft, ctop     = center_x-radius, center_y-radius
        cright, cbottom = center_x+radius, center_y+radius

        # trivial reject if bounding boxes do not intersect
        if rright < cleft or rleft > cright or rbottom < ctop or rtop > cbottom:
            return False  # no collision possible

        # check whether any point of rectangle is inside circle's radius
        for x in (rleft, rleft+width):
            for y in (rtop, rtop+height):
                # compare distance between circle's center point and each point of
                # the rectangle with the circle's radius
                if math.hypot(x-center_x, y-center_y) <= radius:
                    return True  # collision detected

        # check if center of circle is inside rectangle
        if rleft <= center_x <= rright and rtop <= center_y <= rbottom:
            return True  # overlaid

        return False  # no collision detected
