#include "pathfinder/dynamic_barriers_manager.h"

#include "pathfinder/BarriersSubscribers/processing_belt_interpreter_subscriber.h"

#include <utility>

using namespace std;

DynamicBarriersManager::DynamicBarriersManager()
{
    //
}

bool DynamicBarriersManager::hasBarriers(const Point& pos)
{
    for (const auto& subscriber : subscribers)
        if (subscriber->hasBarrier(pos))
            return true;
    return false;
}

void DynamicBarriersManager::addBarrierSubscriber(unique_ptr<AbstractBarriersSubscriber> && subscriber)
{
    subscribers.push_back(std::move(subscriber));
}

