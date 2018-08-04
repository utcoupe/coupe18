#include "engine_constants.h"
#include <math>


float CollisionThresholds::get_stop_distance(float linear_speed)
{
    if (!linear_speed) return 0;
    float dist = CollisionThresholds::STOP_GAIN * linear_speed;
    if (abs(dist) < CollisionThresholds::STOP_MIN) return CollisionThresholds::STOP_MIN;
    if (abs(dist) > CollisionThresholds::STOP_MAX) return CollisionThresholds::STOP_MAX;
    return dist;
}