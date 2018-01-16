class CheckZone(object):
    def __init__(self, width, collision_level):
        self._width = width
        self.collision_level = collision_level

    def _get_shapes(self):
        pass

    def check_collisions(self, obstacles):
        pass


class MainCheckZone(CheckZone):
    def __init__(self, width, collision_level):
        super(MainCheckZone, self).__init__(width, collision_level)

    def _get_shapes(self):
        pass

    def check_collisions(self, obstacles):
        for main_shape in self._get_shapes():
            pass


class PathCheckZone(CheckZone):
    def __init__(self, collision_level):
        super(PathCheckZone, self).__init__(collision_level)
        self._path = []

    def _get_shapes(self):
        pass

    def check_collisions(self, obstacles):
        for path_shape in self._get_shapes():
            pass
