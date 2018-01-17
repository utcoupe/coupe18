class Velocity(object):
    def __init__(self, linear, angular):
        self.linear = linear
        self.angular = angular


class Point(object):
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def tuple2(self):
        return (self.x, self.y)


class Position(Point):
    def __init__(self, x, y, angle = 0.0):
        super(Position, self).__init__(x, y)
        self.a = angle

    def tuple3(self):
        return (self.x, self.y, self.a)
