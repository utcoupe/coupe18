import math
import time

from collisions_shapes_attrib import Point

class MapObstacle(object):
    def __init__(self, position, velocity = None):
        self.Position = position
        self.Velocity = velocity
        self.SpawnTime = time.time()


class CircleObstacle(MapObstacle):
    def __init__(self, position, radius, velocity = None):
        super(CircleObstacle, self).__init__(position, velocity)
        self.Radius = radius

    def __repr__(self):
        return "circle"


class RectObstacle(MapObstacle):
    def __init__(self, position, width, height, velocity = None):
        super(RectObstacle, self).__init__(position, velocity)
        self.Width = width
        self.Height = height

    def corners(self): # Calculs persos, a verifier DEPRECATED
        corners = []
        l = math.sqrt((self.Width / 2.0) ** 2 + (self.Height / 2.0) ** 2)
        corner_phi = math.atan2(self.Height, self.Width)
        for angle_phi in [0, math.pi]:
            for i in range(2):
                phi = angle_phi  + ((-1) ** (i + 1)) * corner_phi
                corners.append(Point(self.Position.X + l * math.cos(phi + self.Position.A), self.Position.Y + l * math.sin(phi + self.Position.A)))
        return corners

    def segments(self):
        c = self.corners()
        return [(c[i], c[(i + 1) % 4]) for i in range(len(c))]

    def __repr__(self):
        return "rect"
