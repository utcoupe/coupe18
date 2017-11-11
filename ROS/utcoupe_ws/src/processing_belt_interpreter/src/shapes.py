from math import atan2, pi


class Shape(object):
    """Class to test collisions with shapes returned by the map"""

    def __init__(self):
        super(Shape, self).__init__()

    def contains(self, x, y):
        pass


class Circle(Shape):
    def __init__(self, x, y, r):
        super(Circle, self).__init__()
        self.x = x
        self.y = y
        self.r = r

    def contains(self, x, y):
        return (x - self.x)**2 + (y - self.y)**2 <= self.r**2


class Polygon(Shape):
    """Thanks to eboix (https://stackoverflow.com/users/1063041/eboix)
    https://stackoverflow.com/questions/10673740/how-to-check-if-a-point-x-y-is-inside-a-polygon-in-the-cartesian-coordinate-sy
    This is not an optimized algorithm and may be slow (TODO ?)"""

    def __init__(self, vertices):
        super(Polygon, self).__init__()
        self.Vertices = vertices

    def toAngle(self, x1, y1, x2, y2):
        theta1 = atan2(y1, x1)
        theta2 = atan2(y2, x2)
        dtheta = theta2 - theta1
        while dtheta > pi:
            dtheta -= 2*pi

        while dtheta < -pi:
            dtheta += 2*pi

        return dtheta

    def contains(self, x, y):
        angle = 0

        for i, v in enumerate(self.Vertices):
            nextv = self.Vertices[(i+1) % len(self.Vertices)]
            angle += self.toAngle(v.x - x, v.y - y, nextv.x - x, nextv.y - y)

        return abs(angle) >= PI


class Rectangle(Shape):
    #  x, y : bottom left
    def __init__(self, x, y, w, h):
        super(Rectangle, self).__init__()
        self.x = x
        self.y = y
        self.w = w
        self.h = h

    def contains(self, x, y):
        return x >= self.x and x <= self.x + self.w and y >= self.y \
                           and y <= self.y + self.h
