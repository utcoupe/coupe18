class CollisionLevel(object):
    LEVEL_STOP      = 0
    LEVEL_DANGER    = 1
    LEVEL_POTENTIAL = 2

class CollisionType(object):
    TYPE_STATIC  = 0
    TYPE_DYNAMIC = 1

class CollisionThresholds(object):
    STOP_MIN  = 0.15 # minimum distance when linear_speed != 0.
    STOP_GAIN = 0.6  # distance in meters when the robot is at 1 m/sec (linear coefficient).
    STOP_MAX  = 1.0  # maximum distance when linear_speed != 0.

    @staticmethod
    def getStopDistance(linear_speed):
        if not linear_speed:
            return 0.0 # If robot stopped, don't create stop rect distance.
        return sorted((CollisionThresholds.STOP_MIN, CollisionThresholds.STOP_GAIN * linear_speed, CollisionThresholds.STOP_MAX))[1] # clamp value
