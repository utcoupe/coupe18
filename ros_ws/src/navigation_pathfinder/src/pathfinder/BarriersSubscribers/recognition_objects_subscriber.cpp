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
