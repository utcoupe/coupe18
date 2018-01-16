class Velocity(object):
    def __init__(self, linear, angular):
        self.Linear = linear
        self.Angular = angular


class Point(object):
    def __init__(self, x, y):
        self.X = x
        self.Y = y


class Position(Point):
    def __init__(self, x, y, angle = 0.0):
        super(Position, self).__init__(x, y)
        self.A = angle

    def tuple2(self):
        return (self.X, self.Y)
