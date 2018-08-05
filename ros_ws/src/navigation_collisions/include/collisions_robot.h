#ifndef COLLISIONS_ROBOT_H
#define COLLISIONS_ROBOT_H

#include "collisions_engine/engine.h"
#include "collisions_engine/engine_check_zone.h"
#include <vector>

enum class NavStatus { STATUS_IDLE, STATUS_NAVIGATING };

class Robot
{
private:
    Position _position;
    Velocity _velocity;
    NavStatus _nav_status;

    float _get_max_main_dist();
public:
    float width, height;
    Robot(float width, float height);
    ~Robot();

    void update_position(Position new_position);
    void update_velocity(float linear, float angular);
    void update_status(NavStatus new_status);
    void update_waypoints(std::vector<Position> new_waypoints);

    void get_main_shapes();
    void get_path_shapes();

    std::vector<Collision> check_collisions(std::vector<MapObstacle> obstacles);
};

#endif