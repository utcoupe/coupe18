#ifndef COLLISIONS_ROBOT_H
#define COLLISIONS_ROBOT_H

#include "collisions_engine/position.h"

enum class NavStatus { STATUS_IDLE, STATUS_NAVIGATING };

class Robot
{
private:
    CollisionsEngine::Position _position;
    CollisionsEngine::Velocity _velocity;
    NavStatus _nav_status;

    float _get_max_main_dist();
public:
    float width, height;
    Robot(float width, float height);
    ~Robot();

    void update_position();//args
    void update_velocity(float linear, float angular);
    void update_status(NavStatus new_status);
    void update_waypoints();//args

    void get_main_shapes();
    void get_path_shapes();

    void check_collisions();
};

#endif