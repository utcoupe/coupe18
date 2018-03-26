#include "pathfinder/BarriersSubscribers/recognition_objects_classifier_subscriber.h"

using namespace std;
using namespace Recognition;

ObjectsClassifierSubscriber::ObjectsClassifierSubscriber(const double& safetyMargin)
    : AbstractBarriersSubscriber(safetyMargin)
{
    //
}

bool ObjectsClassifierSubscriber::hasBarrier(const geometry_msgs::Pose2D& pos)
{
    lock_guard<mutex> lock(g_mutex);
    
    // Rectangles
    for (const auto& rect : lastRects)
        if (isInsideRect(rect, pos))
            return true;
    
    // Circles
    for (const auto& circ : lastCircles)
        if (isInsideCircle(circ, pos))
            return true;
    
    return false;
}

void ObjectsClassifierSubscriber::subscribe(ros::NodeHandle& nodeHandle, std::size_t sizeMaxQueue, std::string topic)
{
    subscriber = nodeHandle.subscribe(
        topic,
        sizeMaxQueue,
        &Recognition::ObjectsClassifierSubscriber::objectsCallback,
        this
    );
}

void ObjectsClassifierSubscriber::objectsCallback(const Objects::ConstPtr& msg)
{
    lock_guard<mutex> lock(g_mutex);
    lastRects.clear();
    addRects(msg->map_rects);
    addRects(msg->unknown_rects);
    lastCircles.clear();
    addCircles(msg->map_circles);
    addCircles(msg->unknown_circles);
}

void ObjectsClassifierSubscriber::addRects(const std::vector<Rectangle>& rects)
{
    lastRects.insert(lastRects.end(), rects.begin(), rects.end());
}

void ObjectsClassifierSubscriber::addCircles(const std::vector<Circle>& circs)
{
    lastCircles.insert(lastCircles.end(), circs.begin(), circs.end());
}

bool ObjectsClassifierSubscriber::isInsideRect(const Rectangle& rect, const geometry_msgs::Pose2D& pos) const
{
    if (pos.x + _safetyMargin < (rect.x - rect.w/2) || pos.x - _safetyMargin > (rect.x + rect.w/2))
        return false;
    if (pos.y + _safetyMargin < (rect.y - rect.h/2) || pos.y - _safetyMargin > (rect.y + rect.h/2))
        return false;
    return true;
}

bool Recognition::ObjectsClassifierSubscriber::isInsideCircle(const Circle& circ, const geometry_msgs::Pose2D& pos) const
{
    processing_lidar_objects::CircleObstacle circle(circ.circle);
    double distToCenter = sqrt(pow(pos.x - circle.center.x, 2) + pow(pos.y - circle.center.y, 2));
    if (distToCenter + _safetyMargin <= circle.radius)
        return true;
    return false;
}
