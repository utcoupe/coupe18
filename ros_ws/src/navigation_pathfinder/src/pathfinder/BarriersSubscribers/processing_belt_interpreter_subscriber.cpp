#include "pathfinder/BarriersSubscribers/processing_belt_interpreter_subscriber.h"

using namespace Processing;
using namespace std;

BeltInterpreterSubscriber::BeltInterpreterSubscriber ()
{
    //
}


bool BeltInterpreterSubscriber::hasBarrier(const geometry_msgs::Pose2D& pos) const
{
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
    lastRectangles.insert(lastRectangles.end(), rects.begin(), rects.end());
}

bool Processing::BeltInterpreterSubscriber::isInsideRect(const geometry_msgs::Pose2D& pos, const Rectangle& rect) const
{
    if (pos.x < rect.x || pos.x > (rect.x + rect.w))
        return false;
    if (pos.y < rect.y || pos.y > (rect.y + rect.h))
        return false;
    return true;
}
