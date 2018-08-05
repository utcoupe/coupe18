#include "ros/ros.h"


Robot::Robot(float width, float height)
{
    Robot::width = width;
    Robot::height = height;

    Robot::_position = Position(0, 0);
    Robot::_velocity = Velocity(width, height);
    Robot::_nav_status = NavStatus::STATUS_IDLE;

    Robot::_path_check_zone = PathCheckZone(width, height, CollisionLevel::LEVEL_DANGER);
}

void Robot::update_position(Position new_position)
{
    _position = new_position;
}

void Robot::update_velocity(float linear, float angular)
{
    _velocity.linear = linear;
    _velocity.angular = angular;
}

void Robot::update_status(NavStatus new_status)
{
    _nav_status = new_status;
}

void Robot::update_waypoints(std::vector<Position> new_waypoints)
{
    _path_check_zone.update_waypoints(new_waypoints);
}

void Robot::get_main_shapes()
{
    return _velocity.get_shapes(_position, _get_max_main_dist());
}

void Robot::get_path_shapes()
{
    return _path_check_zone.get_shapes(_position)
}

std::vector<Collision> Robot::check_collisions(std::vector<MapObstacle> obstacles)
{
    VelocityCheckZone _check_zone = new VelocityCheckZone(width, height, CollisionLevel::LEVEL_STOP);
    std::vector<Collision> c1 = _check_zone.check_collisions(_position, obstacles);
    std::vector<Collision> c2 = _path_check_zone.check_collisions(_position, obstacles);
    // TODO remove duplicate collisions between the two

    std::vector<MapObstacle> collisions;
    collisions.reserve(c1.size() + c2.size()); // preallocate memory
    collisions.insert(collisions.end(), c1.begin(), c1.end());
    collisions.insert(collisions.end(), c2.begin(), c2.end());
    return collisions;
}

float Robot::_get_max_main_dist()
{
    if(_path_check_zone.waypoints.size() > 0)
    {
        Position w = self._path_check_zone.waypoints[0];
        return sqrt(pow(w.x - self._position.x, 2) + pow(w.y - self._position.y, 2));
    }
    else return -1;
}
