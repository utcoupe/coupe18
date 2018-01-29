#ifndef ABSTRACT_BARRIERS_SUBSCRIBER
#define ABSTRACT_BARRIERS_SUBSCRIBER

#include <ros/ros.h>

#include "geometry_msgs/Pose2D.h"

class AbstractBarriersSubscriber
{
public:
    AbstractBarriersSubscriber(const double& safetyMargin) : _safetyMargin(safetyMargin) {};
    
    virtual bool hasBarrier(const geometry_msgs::Pose2D& pos) const = 0;
    virtual void subscribe(ros::NodeHandle& nodeHandle, std::size_t sizeMaxQueue, std::string topic) = 0;
    void setSafetyMargin(const double& safetyMargin) { _safetyMargin = _safetyMargin; }

protected:
    ros::Subscriber subscriber;
    double _safetyMargin;
};

#endif // ABSTRACT_BARRIERS_SUBSCRIBER
