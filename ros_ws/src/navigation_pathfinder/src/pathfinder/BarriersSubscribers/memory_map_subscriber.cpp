#include "pathfinder/BarriersSubscribers/memory_map_subscriber.h"

#include "memory_map/MapGetObjects.h"

using namespace Memory;
using namespace std;
using json = nlohmann::json;

MapSubscriber::MapSubscriber(const double& safetyMargin)
    : AbstractBarriersSubscriber(safetyMargin)
{
}

bool MapSubscriber::hasBarrier(const geometry_msgs::Pose2D& pos)
{
    return false;
}

void MapSubscriber::subscribe(ros::NodeHandle& nodeHandle, [[maybe_unused]] std::size_t sizeMaxQueue, std::string topic)
{
    srvGetMapObjects = nodeHandle.serviceClient<memory_map::MapGetObjects>(topic);
}

void MapSubscriber::fetchOccupancyData()
{
    memory_map::MapGetObjects srv;
    srv.request.collisions_only = true;
    if (!srvGetMapObjects.call(srv) || !srv.response.success)
    {
        ROS_ERROR("Error when trying to call memory_map/MapGetObjects");
        return;
    }
    lastReceivedJson = json::parse(string(srv.response.objects.data()));
    ROS_DEBUG_STREAM("Received from map: " << lastReceivedJson.dump(4));
}
