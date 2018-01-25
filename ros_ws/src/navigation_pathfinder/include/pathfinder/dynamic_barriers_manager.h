#ifndef DYNAMIC_BARRIERS_MANAGER_H
#define DYNAMIC_BARRIERS_MANAGER_H

#include "pathfinder/point.h"
#include "pathfinder/BarriersSubscribers/abstract_barriers_subscriber.h"

#include <memory>
#include <vector>

class DynamicBarriersManager
{
public:
    DynamicBarriersManager(size_t height, size_t width);
    
    bool hasBarriers(const Point& pos);
    
private:
    std::vector< std::unique_ptr<AbstractBarriersSubscriber> > subscribers;
};

#endif // DYNAMIC_BARRIERS_MANAGER_H
