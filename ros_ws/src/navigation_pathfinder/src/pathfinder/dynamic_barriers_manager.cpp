#include "pathfinder/dynamic_barriers_manager.h"

#include "pathfinder/BarriersSubscribers/processing_belt_interpreter_subscriber.h"
#include "pathfinder/BarriersSubscribers/processing_lidar_objects_subscriber.h"

#include <utility>

using namespace std;

DynamicBarriersManager::DynamicBarriersManager()
{
    //
}

bool DynamicBarriersManager::hasBarriers(const geometry_msgs::Pose2D& pos)
{
    for (const auto& subscriber : subscribers)
        if (subscriber->hasBarrier(pos))
            return true;
    return false;
}

bool DynamicBarriersManager::hasBarriers(const Point& pos)
{
    return hasBarriers(pointToPose2D(pos));
}


void DynamicBarriersManager::addBarrierSubscriber(unique_ptr<AbstractBarriersSubscriber> && subscriber)
{
    subscribers.push_back(std::move(subscriber));
}

void DynamicBarriersManager::setConvertor(std::shared_ptr<PosConvertor> convertor)
{
    _convertor = convertor;
}

void DynamicBarriersManager::updateSafetyMargin(const double& newMargin)
{
    for (auto& subscriber : subscribers)
        subscriber->setSafetyMargin(newMargin);
}


geometry_msgs::Pose2D DynamicBarriersManager::pointToPose2D(const Point& pos) const
{
    auto convertedPos = _convertor->fromMapToRosPos(pair<double, double>(pos.getX(), pos.getY()));
    geometry_msgs::Pose2D newPos;
    newPos.x = convertedPos.first;
    newPos.y = convertedPos.second;
    return newPos;
}

