#include "engine_check_zone.h"
#include <math>

// Velocity class
Velocity::Velocity(float width, float height, float linear, float angular)
{
    Velocity::linear = linear;
    Velocity::angular = angular;

    _check_zone = VelocityCheckZone(width, height, CollisionLevel::LEVEL_STOP);
}

std::vector<MapObstacle> Velocity::get_shapes(Position object_pos, float max_dist = -1)
{
    return _check_zone.get_shapes(object_pos, linear, angular, max_dist);
}

std::vector<Collision> Velocity::check_collisions(Position object_pos, std::vector<MapObstacle> obstacles)
{
    return _check_zone.check_collisions(object_pos, linear, angular, obstacles)
}

// CheckZone class
CheckZone::CheckZone(float width, float height, CollisionLevel collision_level)
{
    CheckZone::width = width;
    CheckZone::height = height;
    CheckZone::collision_level = collision_level;
}

// VelocityCheckZone class
std::vector<MapObstacle> VelocityCheckZone::get_shapes(Position robot_pos, float vel_linear, 
                                                       float vel_angular, float max_dist = -1)
{
    std::vector<MapObstacle> shapes;

    if(abs(vel_linear) < CollisionThresholds.VEL_MIN) // if the object isn't moving fast enough, don't create the rect.
        return shapes;

    float expansion_dist = CollisionThresholds.get_stop_distance(vel_linear);
    if(max_dist != -1)
        expansion_dist = min(expansion_dist, max_dist); // If set, reduce the expansion to the provided limit.
    
    float w = height + expansion_dist;
    float h = width;
    float l = w / 2.0 - height / 2.0;
    float side_a = 0;
    if(vel_linear < 0) side_a = math.pi;

    shapes.push_back((Position(robot_pos.x + l * cos(robot_pos.a + side_a),
                               robot_pos.y + l * sin(robot_pos.a + side_a),
                               robot_pos.a), w, h));
    return shapes;
}

std::vector<Collision> VelocityCheckZone::check_collisions(Position robot_pos, float vel_linear, 
                                                           float vel_angular, std::vector<MapObstacle> obstacles)
{
    std::vector<Collision> collisions;
    std::vector<MapObstacle> intersections = CollisionsResolver.find_collisions(
                        self.get_shapes(robot_pos, vel_linear, vel_angular), obstacles);
    for(int i = 0; i < intersections.size(); i++)
    {
        float approx_d = math.sqrt((robot_pos.x - o.position.x) ** 2 + \
                                   (robot_pos.y - o.position.y) ** 2); // TODO Totally unprecise distance
        collisions.push_back(Collision(CollisionLevel::LEVEL_STOP, o, approx_d));
    }    
    return collisions;
}


// PathCheckZone class
PathCheckZone::PathCheckZone(float width, float height, CollisionLevel collision_level)
    :CheckZone(width, height, collision_level) { }

std::vector<MapObstacle> PathCheckZone::get_shapes(Position robot_pos)
{
    std::vector<MapObstacle> shapes;
    if(waypoints.size() >= 1)
    {
        std::vector<Position> path = _get_full_path(robot_pos);
        for(int i = 0; i < path.size(); i++)
        {
            // Creating a rectangle with the robot's width between each waypoint
            Position p_w = (i == 0) ? robot_pos : path[i - 1];
            Position w = path[i];

            if(p_w.x != w.x && p_w.y != w.y)
            {
                float d = sqrt(pow(w.x - p_w.x, 2) + pow(w.y - p_w.y, 2));
                float angle = atan((w.y - p_w.y) / (w.x - p_w.x));
                Position pos = Position((w.x + p_w.x) / 2.0, (w.y + p_w.y) / 2.0, angle);

                shapes.push_back(RectObstacle(pos, d, width));
                if(i == path.size() - 1)
                    shapes.push_back(RectObstacle(Position(w.x, w.y, angle), height, width));
                else
                {
                    float r = sqrt(pow(width, 2) + pow(height, 2)) / 2.0;
                    shapes.push_back(CircleObstacle(Position(w.x, w.y), r));
                }
            }
        }
    }
    return shapes;
}

std::vector<Collision> PathCheckZone::check_collisions(Position robot_pos, std::vector<MapObstacle> obstacles)
{
    std::vector<Collision> collisions;
    std::vector<MapObstacle> intersections = CollisionsResolver.find_collisions(get_shapes(robot_pos), obstacles);

    for(int i = 0; i < intersections.size(); i++)
    {
        float approx_d = sqrt(pow(robot_pos.x - o.position.x, 2) + pow(robot_pos.y - o.position.y, 2)); // Imprecise
        collisions.push_back(Collision((approx_d < CollisionThresholds::DANGER_RADIUS) ? CollisionLevel.LEVEL_DANGER : CollisionLevel.LEVEL_POTENTIAL,
                                        o, approx_d));
    }
    return collisions;
}

void PathCheckZone::update_waypoints(std::vector<Position> new_waypoints)
{
    if(new_waypoints.size() > 0) waypoints = new_waypoints;
    else ROS_ERR("Trying to update the robot path with an invalid new path.");
}

std::vector<Position> PathCheckZone::_get_full_path(Position robot_pos)
{ // add the current robot position at the start of the path
    std::vector<Position> new_waypoints = waypoints;
    std::reverse(new_waypoints.begin(), new_waypoints.end()); //TODO more efficient way ?
    new_waypoints.push_back(robot_pos);
    std::reverse(new_waypoints.begin(), new_waypoints.end());
    return new_waypoints;
}
