#include "pathfinder/BarriersSubscribers/processing_lidar_objects_subscriber.h"
#include <cmath>

using namespace Processing;
using namespace std;

LidarObjectsSubscriber::LidarObjectsSubscriber(const double& safetyMargin)
    : AbstractBarriersSubscriber(safetyMargin)
{
    //
}

bool LidarObjectsSubscriber::hasBarrier(const geometry_msgs::Pose2D& pos)
{
    lock_guard<mutex> lock(g_mutex);
    for (const auto& circle : lastCircles)
        if (isInsideCircle(pos, circle))
            return true;
    return false;
}

void LidarObjectsSubscriber::subscribe(ros::NodeHandle& nodeHandle, std::size_t sizeMaxQueue, std::string topic)
{
    subscriber = nodeHandle.subscribe(
        topic,
        sizeMaxQueue,
        &Processing::LidarObjectsSubscriber::obstaclesCallback,
        this
    );
}

void LidarObjectsSubscriber::obstaclesCallback(const processing_lidar_objects::Obstacles::ConstPtr& msg)
{
    lock_guard<mutex> lock(g_mutex);
    lastCircles.clear();
    lastCircles.insert(lastCircles.end(), msg->circles.begin(), msg->circles.end());
}

bool LidarObjectsSubscriber::isInsideCircle(const geometry_msgs::Pose2D& pos, const Circle& circle) const
{
    double distToCenter = sqrt(pow(pos.x - circle.center.x, 2) + pow(pos.y - circle.center.y, 2));
    if (distToCenter + _safetyMargin <= circle.radius)
        return true;
    return false;
}
