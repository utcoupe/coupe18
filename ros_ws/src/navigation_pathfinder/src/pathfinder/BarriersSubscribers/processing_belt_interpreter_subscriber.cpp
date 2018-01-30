#include "pathfinder/BarriersSubscribers/processing_belt_interpreter_subscriber.h"

using namespace Processing;
using namespace std;

BeltInterpreterSubscriber::BeltInterpreterSubscriber (const double& safetyMargin):
    AbstractBarriersSubscriber(safetyMargin)
{
    //
}


bool BeltInterpreterSubscriber::hasBarrier(const geometry_msgs::Pose2D& pos)
{
    lock_guard<mutex> lock(g_mutex);
    for (const auto& rect : lastRectangles)
        if (isInsideRect(pos, rect))
            return true;
    return false;
}

void Processing::BeltInterpreterSubscriber::subscribe(ros::NodeHandle& nodeHandle, size_t sizeMaxQueue, string topic)
{
    subscriber = nodeHandle.subscribe(
        topic,
        sizeMaxQueue,
        &Processing::BeltInterpreterSubscriber::rectsFilteredTopicCallback,
        this
    );
}


void BeltInterpreterSubscriber::rectsFilteredTopicCallback(const processing_belt_interpreter::BeltFiltered::ConstPtr& msg)
{
    lastRectangles.clear();
    addRects(msg->map_rects);
    addRects(msg->unknown_rects);
}

void Processing::BeltInterpreterSubscriber::addRects(const vector<Rectangle>& rects)
{
    lock_guard<mutex> lock(g_mutex);
    lastRectangles.insert(lastRectangles.end(), rects.begin(), rects.end());
}

bool Processing::BeltInterpreterSubscriber::isInsideRect(const geometry_msgs::Pose2D& pos, const Rectangle& rect) const
{
    if (pos.x + _safetyMargin < rect.x || pos.x - _safetyMargin > (rect.x + rect.w))
        return false;
    if (pos.y + _safetyMargin < rect.y || pos.y - _safetyMargin > (rect.y + rect.h))
        return false;
    return true;
}
