#ifndef DYNAMIC_BARRIERS_MANAGER_H
#define DYNAMIC_BARRIERS_MANAGER_H

#include "pathfinder/point.h"

#include <vector>

class DynamicBarriersManager
{
public:
    DynamicBarriersManager(size_t height, size_t width);
    
    bool hasBarriers(const Point& pos);
    
private:
    std::vector< std::vector<bool> > occupancy; // TODO May change in the future, depending of the needs.
    // TODO Add enemy_tracker client
};

#endif // DYNAMIC_BARRIERS_MANAGER_H
