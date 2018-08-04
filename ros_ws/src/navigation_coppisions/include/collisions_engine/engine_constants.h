#ifndef ENGINE_CONSTANTS_H
#define ENGINE_CONSTANTS_H

enum class CollisionLevel
{
    LEVEL_STOP, 
    LEVEL_DANGER, 
    LEVEL_POTENTIAL
};

enum class CollisionType
{
    TYPE_STATIC,
    TYPE_DYNAMIC
};

class CollisionThresholds
{
public:
    static const float VEL_MIN   = 0.05;   // minimum linear speed (m/s) before creating a stop rect.
    static const float STOP_MIN  = 0.1;    // minimum distance when linear_speed != 0.
    static const float STOP_GAIN = 0.6;    // distance in meters when the robot is at 1 m/sec (linear coefficient).
    static const float STOP_MAX  = 1.0;    // maximum distance when linear_speed != 0.

    static const float DANGER_RADIUS = 0.8; // radius around the robot where obstacles are considered at LEVEL_DANGER vs. LEVEL_POTENTIAL.
    
    static float get_stop_distance(float linear_speed);
private:
    CollisionThresholds();
};

#endif
