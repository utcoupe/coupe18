#ifndef ENGINE_CHECK_ZONE_H
#define ENGINE_CHECK_ZONE_H

#include "engine.h"
#include "engine_shapes.h"
#include "engine_constants.h"
#include <vector>


class CheckZone
{
public:
    float width, height;
    CollisionLevel collision_level;

    CheckZone(float width, float height, CollisionLevel collision_level);
    ~CheckZone();

    virtual std::vector<MapObstacle> get_shapes(Position robot_pos);
    virtual std::vector<Collision> check_collisions(Position robot_pos, std::vector<MapObstacle> obstacles);
};


class VelocityCheckZone: public CheckZone
{
public:
    VelocityCheckZone(float width, float height, CollisionLevel collision_level);
    ~VelocityCheckZone();

    std::vector<MapObstacle> get_shapes(Position robot_pos, float vel_linear, 
                                        float vel_angular, float max_dist = -1);
    std::vector<Collision> check_collisions(Position robot_pos, float vel_linear, 
                                            float vel_angular, std::vector<MapObstacle> obstacles);
};


class PathCheckZone: public CheckZone
{
public:
    PathCheckZone(float width, float height, CollisionLevel collision_level);
    ~PathCheckZone();

    std::vector<MapObstacle> get_shapes(Position robot_pos);
    std::vector<Collision> check_collisions(Position robot_pos, std::vector<MapObstacle> obstacles);

    void update_waypoints(std::vector<Position> new_waypoints);

private:
    std::vector<Position> waypoints;
    std::vector<Position> _get_full_path(Position robot_pos);
};


#endif