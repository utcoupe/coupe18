
#include "../include/objects_listener.h"

void ObjectsListener::on_belt_callback(const BeltRectsConstPtr &rects)
{
    this->rects = *rects.get();
}

void ObjectsListener::on_lidar_callback(const ObstaclesConstPtr &obstacles)
{
    this->lidar_obstacles = *obstacles.get();
}