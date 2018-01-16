from engine_constants import CollisionLevel
from engine_shapes_attrib import Point, Position
from engine_check_zone import PathCheckZone, MainCheckZone


class NavStatus(object):
    STATUS_IDLE       = 0
    STATUS_NAVIGATING = 1


class Robot(object):
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self._position = Position(0, 0)
        self._nav_status = NavStatus.STATUS_IDLE

        self._main_check_zone = MainCheckZone(self.width, CollisionLevel.LEVEL_STOP)
        self._path_check_zone = PathCheckZone(self.width, CollisionLevel.LEVEL_DANGER)

    def update_position(self, tuple3):
        self._position.X = tuple3[0]
        self._position.Y = tuple3[1]
        self._position.A = tuple3[2]

    def update_status(self, new_status):
        self._nav_status = new_status

    def check_collisions(self, obstacles):
        return self._main_check_zone.check_collisions(obstacles) + \
               self._path_check_zone.check_collisions(obstacles)
        # TODO remove duplicate collisions between the two
