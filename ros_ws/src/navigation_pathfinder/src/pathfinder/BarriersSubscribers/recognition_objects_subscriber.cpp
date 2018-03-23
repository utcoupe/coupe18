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
    return false;
}

void ObjectsClassifierSubscriber::subscribe(ros::NodeHandle& nodeHandle, std::size_t sizeMaxQueue, std::string topic)
{
    //
}
