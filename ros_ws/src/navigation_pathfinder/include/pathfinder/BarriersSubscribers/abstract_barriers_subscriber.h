#ifndef ABSTRACT_BARRIERS_SUBSCRIBER
#define ABSTRACT_BARRIERS_SUBSCRIBER

#include <ros/ros.h>

#include "geometry_msgs/Pose2D.h"

class AbstractBarriersSubscriber
{
public:
    AbstractBarriersSubscriber() {};
    
    virtual bool hasBarrier(const geometry_msgs::Pose2D& pos) const = 0;
    virtual void subscribe(ros::NodeHandle& nodeHandle, std::size_t sizeMaxQueue, std::string topic) = 0;

protected:
    ros::Subscriber subscriber;
};

#endif // ABSTRACT_BARRIERS_SUBSCRIBER
