#ifndef MEMORY_MAP_SUBSCRIBER_H
#define MEMORY_MAP_SUBSCRIBER_H

#include "pathfinder/BarriersSubscribers/abstract_barriers_subscriber.h"

#include "nlohmann/json.hpp"

#include <vector>

namespace Memory {
    class MapSubscriber : public AbstractBarriersSubscriber
    {
    public:
        MapSubscriber(const double& safetyMargin);
        
        bool hasBarrier(const geometry_msgs::Pose2D& pos) override;
        void subscribe(ros::NodeHandle& nodeHandle, std::size_t sizeMaxQueue, std::string topic) override;
        
        void fetchOccupancyData() override;
        
    private:
        std::vector<nlohmann::json> lastReceivedJsons;
        ros::ServiceClient srvGetMapObjects;
        
        bool isInsideRectangle(const geometry_msgs::Pose2D& pos, const nlohmann::json jsonRect) const;
        bool isInsideCircle(const geometry_msgs::Pose2D& pos, const nlohmann::json jsonCircle) const;
    };
}

#endif // MEMORY_MAP_SUBSCRIBER_H
