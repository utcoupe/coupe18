#include "ros/ros.h"


Robot::Robot(float width, float height)
{
    Robot::width = width;
    Robot::height = height;

    Robot::_position = Position(0, 0);
    Robot::_velocity = Velocity(width, height);
    Robot::_nav_status = NavStatus::STATUS_IDLE;

    Robot::_path_check_zone = pathCheckZone(width, height, CollisionLevel::LEVEL_DANGER);
}

Robot::~Robot()
{
}