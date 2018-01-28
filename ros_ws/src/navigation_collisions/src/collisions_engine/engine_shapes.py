import math
import time

from engine_shapes_attrib import Point, Position

class MapObstacle(object):
    def __init__(self, position, velocity = None):
        self.position = position
        self.pelocity = velocity
        self.spawn_time = time.time()


class CircleObstacle(MapObstacle):
    def __init__(self, position, radius, velocity = None):
        super(CircleObstacle, self).__init__(position, velocity)
        self.radius = radius

    def __repr__(self):
        return "circle"


class RectObstacle(MapObstacle):
    def __init__(self, position, width, height, velocity = None):
        super(RectObstacle, self).__init__(position, velocity)
        self.width = width
        self.height = height

    def corners(self): # Calculs persos, a verifier DEPRECATED
        corners = []
        l = math.sqrt((self.width / 2.0) ** 2 + (self.height / 2.0) ** 2)
        corner_phi = math.atan2(self.height, self.width)
        for angle_phi in [0, math.pi]:
            for i in range(2):
                phi = angle_phi  + ((-1) ** (i + 1)) * corner_phi
                corners.append(Point(self.position.x + l * math.cos(phi + self.position.a), self.position.y + l * math.sin(phi + self.position.a)))
        return corners

    def segments(self):
        c = self.corners()
        return [(c[i], c[(i + 1) % 4]) for i in range(len(c))]

    def __repr__(self):
        return "rect"
