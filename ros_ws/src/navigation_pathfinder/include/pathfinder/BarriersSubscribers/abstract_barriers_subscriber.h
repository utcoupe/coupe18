#ifndef ABSTRACT_BARRIERS_SUBSCRIBER
#define ABSTRACT_BARRIERS_SUBSCRIBER

#include "pathfinder/point.h"

class AbstractBarriersSubscriber
{
public:
    AbstractBarriersSubscriber() {};
    
    virtual bool hasBarrier(const Point& pos) const = 0;
};

#endif // ABSTRACT_BARRIERS_SUBSCRIBER
