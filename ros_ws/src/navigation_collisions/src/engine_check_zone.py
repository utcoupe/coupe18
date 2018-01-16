from engine import CollisionsExplorer


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
        return []

    def check_collisions(self, obstacles):
        collisions = CollisionsExplorer.find_collisions(self._get_shapes(), obstacles)
        return collisions


class PathCheckZone(CheckZone):
    def __init__(self, width, collision_level):
        super(PathCheckZone, self).__init__(width, collision_level)
        self._path = []

    def _get_shapes(self):
        return []

    def check_collisions(self, obstacles):
        collisions = []
        for path_shape in self._get_shapes():
            pass
        return collisions
