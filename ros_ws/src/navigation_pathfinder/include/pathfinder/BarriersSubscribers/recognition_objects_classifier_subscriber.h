#ifndef RECOGNITION_OBJECTS_CLASSIFIER_SUBSCRIBER
#define RECOGNITION_OBJECTS_CLASSIFIER_SUBSCRIBER

#include "abstract_barriers_subscriber.h"

#include "recognition_objects_classifier/ClassifiedObjects.h"

#include <vector>

namespace Recognition
{
    class ObjectsClassifierSubscriber : public AbstractBarriersSubscriber
    {
    public:
        ObjectsClassifierSubscriber(const double& safetyMargin);
        
        bool hasBarrier(const geometry_msgs::Pose2D& pos) override;
        void subscribe(ros::NodeHandle& nodeHandle, std::size_t sizeMaxQueue, std::string topic) override;
        
    private:
        using Objects       = recognition_objects_classifier::ClassifiedObjects;
        using Rectangle    = processing_belt_interpreter::RectangleStamped;
        using Circle       = recognition_objects_classifier::CircleObstacleStamped;
        
        std::vector<Rectangle> lastRects;
        std::vector<Circle> lastCircles;
        
        void objectsCallback(const Objects::ConstPtr& msg);
        void addRects(const std::vector<Rectangle>& rects);
        void addCircles(const std::vector<Circle>& circs);
        
        bool isInsideRect(const Rectangle& rect, const geometry_msgs::Pose2D& pos) const;
        bool isInsideCircle(const Circle& circ, const geometry_msgs::Pose2D& pos) const;
    };
} // namespace Recognition

#endif // RECOGNITION_OBJECTS_CLASSIFIER_SUBSCRIBER
