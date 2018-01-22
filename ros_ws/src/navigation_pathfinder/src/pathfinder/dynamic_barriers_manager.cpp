#include "pathfinder/dynamic_barriers_manager.h"

using namespace std;

DynamicBarriersManager::DynamicBarriersManager(size_t height, size_t width)
{
    occupancy = vector< vector<bool> >(
        height,
        vector<bool>(width, false)
    );
}

bool DynamicBarriersManager::hasBarriers(const Point& pos)
{
    return occupancy[pos.getY()][pos.getX()];
}
