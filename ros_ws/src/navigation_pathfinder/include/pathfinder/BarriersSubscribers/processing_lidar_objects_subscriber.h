#ifndef PROCESSING_LIDAR_OBJECTS_SUBSCRIBER
#define PROCESSING_LIDAR_OBJECTS_SUBSCRIBER

#include "pathfinder/BarriersSubscribers/abstract_barriers_subscriber.h"
#include "processing_lidar_objects/Obstacles.h"
#include "processing_lidar_objects/CircleObstacle.h"

namespace Processing
{
    class LidarObjectsSubscriber : public AbstractBarriersSubscriber
    {
    public:
        LidarObjectsSubscriber(const double& safetyMargin);
        
        bool hasBarrier(const geometry_msgs::Pose2D& pos);
        void subscribe(ros::NodeHandle& nodeHandle, std::size_t sizeMaxQueue, std::string topic);
        
    private:
        typedef processing_lidar_objects::CircleObstacle Circle;
        std::vector<Circle> lastCircles;
        
        void obstaclesCallback(const processing_lidar_objects::Obstacles::ConstPtr& msg);
        bool isInsideCircle(const geometry_msgs::Pose2D& pos, const Circle& circle) const;
    };
}


#endif // PROCESSING_LIDAR_OBJECTS_SUBSCRIBER
