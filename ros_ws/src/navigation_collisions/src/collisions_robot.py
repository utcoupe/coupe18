from collisions_engine import Position, Velocity, CollisionLevel, PathCheckZone, MainCheckZone


class NavStatus(object):
    STATUS_IDLE       = 0
    STATUS_NAVIGATING = 1


class Robot(object):
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self._position = Position(0, 0)
        self._velocity = Velocity(0, 0)
        self._nav_status = NavStatus.STATUS_IDLE

        self._main_check_zone = MainCheckZone(self.width, self.height, CollisionLevel.LEVEL_STOP)
        self._path_check_zone = PathCheckZone(self.width, self.height, CollisionLevel.LEVEL_DANGER)

    def update_position(self, tuple3):
        self._position.X = tuple3[0]
        self._position.Y = tuple3[1]
        self._position.A = tuple3[2]

    def update_velocity(self, linear, angular):
        self._velocity.linear = linear
        self._velocity.angular = angular

    def update_status(self, new_status):
        self._nav_status = new_status

    def update_path(self, new_waypoints):
        self._path_check_zone.update_path(new_waypoints)

    def get_main_shapes(self):
        return self._main_check_zone.get_shapes(self._position, self._velocity)

    def get_path_shapes(self):
        return self._path_check_zone.get_shapes(self._position)

    def check_collisions(self, obstacles):
        return self._main_check_zone.check_collisions(self._position, self._velocity, obstacles) + \
               self._path_check_zone.check_collisions(self._position, obstacles)
        # TODO remove duplicate collisions between the two
