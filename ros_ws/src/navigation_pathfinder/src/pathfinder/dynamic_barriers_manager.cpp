#include "pathfinder/dynamic_barriers_manager.h"

#include "pathfinder/BarriersSubscribers/processing_belt_interpreter_subscriber.h"

using namespace std;

DynamicBarriersManager::DynamicBarriersManager(size_t height, size_t width)
{
    subscribers.push_back(make_unique<Processing::BeltInterpreterSubscriber>());
}

bool DynamicBarriersManager::hasBarriers(const Point& pos)
{
    for (const auto& subscriber : subscribers)
        if (subscriber->hasBarrier(pos))
            return true;
    return false;
}
