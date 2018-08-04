#include "obstacles_stack.h"
#include <chrono>
#include "stdint.h"

static std::vector<MapObstacle> ObstaclesStack::to_list()
{
    std::vector<MapObstacle> obstacles;
    obstacles.reserve(belt_obstacles.size() + lidar_obstacles.size()); // preallocate memory
    obstacles.insert(obstacles.end(), belt_obstacles.begin(), belt_obstacles.end());
    obstacles.insert(obstacles.end(), lidar_obstacles.begin(), lidar_obstacles.end());
    return obstacles;
}

static void ObstaclesStack::garbage_collect()
{
    uint32_t current_millis = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();

    count_belt = belt_obstacles.size();
    for(int i = 0; i < count_belt; i++)
    {
        if(current_millis - belt_obstacles[i].spawn_time > OBSTACLES_LIFESPAN)
        {
            belt_obstacles.erase(belt_obstacles.begin() + i);
            --count_belt;
        }
    }

    count_lidar = lidar_obstacles.size();
    for(int i = 0; i < count_lidar; i++)
    {
        if(current_millis - lidar_obstacles[i].spawn_time > OBSTACLES_LIFESPAN)
        {
            lidar_obstacles.erase(lidar_obstacles.begin() + i);
            --count_lidar;
        }
    }
}

static void ObstaclesStack::update_belt_obstacles(std::vector<MapObstacle> new_obstacles)
{
    belt_obstacles = new_obstacles;
}

static void ObstaclesStack::update_lidar_obstacles(std::vector<MapObstacle> new_obstacles)
{
    lidar_obstacles = new_obstacles;
}
