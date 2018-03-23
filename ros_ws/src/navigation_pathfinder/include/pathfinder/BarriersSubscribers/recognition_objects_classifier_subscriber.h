#ifndef RECOGNITION_OBJECTS_CLASSIFIER_SUBSCRIBER
#define RECOGNITION_OBJECTS_CLASSIFIER_SUBSCRIBER

#include "abstract_barriers_subscriber.h"

namespace Recognition
{
    class ObjectsClassifierSubscriber : public AbstractBarriersSubscriber
    {
    public:
        ObjectsClassifierSubscriber(const double& safetyMargin);
        
        bool hasBarrier(const geometry_msgs::Pose2D& pos) override;
        void subscribe(ros::NodeHandle& nodeHandle, std::size_t sizeMaxQueue, std::string topic) override;
    };
} // namespace Recognition

#endif // RECOGNITION_OBJECTS_CLASSIFIER_SUBSCRIBER
