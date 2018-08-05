#ifndef OBSTACLES_STACK_H
#define OBSTACLES_STACK_H

#include "collisions_robot.h"
#include <vector>

class Map
{
public:
    static Robot robot;
private:
    Map();
    ~Map();
};



class ObstaclesStack
{
public:
    static std::vector<MapObstacle> belt_obstacles;
    static std::vector<MapObstacle> lidar_obstacles;

    static std::vector<MapObstacle> to_list();
    static void garbage_collect();

    static void update_belt_obstacles(std::vector<MapObstacle> new_obstacles);
    static void update_lidar_obstacles(std::vector<MapObstacle> new_obstacles);
private:
    ObstaclesStack();
    ~ObstaclesStack();
};


#endif