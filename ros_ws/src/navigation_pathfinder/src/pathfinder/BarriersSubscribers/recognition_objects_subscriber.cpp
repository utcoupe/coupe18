#include "pathfinder/BarriersSubscribers/recognition_objects_classifier_subscriber.h"

#include <cmath>

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
    if (rect.h == 0 || rect.w == 0)
        return false;
    double dx, dy; // we want the center of the rectangle as origin
    dx = pos.x - rect.x;
    dy = pos.y - rect.y;
    double a, b; // (a,b) => coordinates of pos with the center of the rectangle as origin and its sides as vectors
    a = -dx*cos(rect.a) - dy*sin(M_PI - rect.a);
    b = dx*sin(rect.a) - dy*cos(rect.a);
    // if a/rect.witdh  is in [-1/2,1/2] and b/rect.height in [-1/2,1/2], then the pos is inside the rectangle
    double da, db;
    da = a/(rect.w + _safetyMargin);
    db = b/(rect.h + 2*_safetyMargin);
    if (da  > 1.0/2.0 || da < -1.0/2.0)
        return false;
    if (db > 1.0/2.0 || db < -1.0/2.0)
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
