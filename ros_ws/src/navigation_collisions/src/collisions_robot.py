from engine_constants import CollisionLevel
from engine_check_zone import PathCheckZone, FrontCheckZone


class Robot(object):
    def __init__(self):
        self._main_check_zone = MainCheckZone(CollisionLevel.LEVEL_STOP)
        self._path_check_zone = PathCheckZone(CollisionLevel.LEVEL_DANGER)

    def check_collisions(self, obstacles):
        return self._main_check_zone.check_collisions(obstacles) + \
               self._path_check_zone.check_collisions(obstacles)
