#include "pathfinder/BarriersSubscribers/processing_belt_interpreter_subscriber.h"

using namespace Processing;
using namespace std;

BeltInterpreterSubscriber::BeltInterpreterSubscriber ()
{
    //
}

bool BeltInterpreterSubscriber::hasBarrier(const Point& pos) const
{
    for (const auto& rect : lastRectangles)
        if (isInsideRect(pos, rect))
            return true;
    return false;
}

void BeltInterpreterSubscriber::rectsFilteredTopicCallback(const processing_belt_interpreter::BeltFiltered::ConstPtr& msg)
{
    lastRectangles.clear();
    addRects(msg->map_rects);
    addRects(msg->unknown_rects);
}

void Processing::BeltInterpreterSubscriber::addRects(const std::vector<Rectangle>& rects)
{
    lastRectangles.insert(lastRectangles.end(), rects.begin(), rects.end());
}

bool Processing::BeltInterpreterSubscriber::isInsideRect(const Point& pos, const Rectangle& rect) const
{
    if (pos.getX() < rect.x || pos.getX() > (rect.x + rect.w))
        return false;
    if (pos.getY() < rect.y || pos.getY() > (rect.y + rect.h))
        return false;
    return true;
}
