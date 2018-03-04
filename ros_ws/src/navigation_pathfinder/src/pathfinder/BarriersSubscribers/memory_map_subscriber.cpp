#include "pathfinder/BarriersSubscribers/memory_map_subscriber.h"

#include "memory_map/MapGetObjects.h"

#include <cmath>

using namespace Memory;
using namespace std;
using json = nlohmann::json;

inline double getNorme2Distance(const double& x1, const double& y1, const double& x2, const double& y2)
{
    double dX = x1 - x2;
    double dY = y1 - y2;
    return sqrt(dX*dX + dY*dY);
}

MapSubscriber::MapSubscriber(const double& safetyMargin)
    : AbstractBarriersSubscriber(safetyMargin)
{
}

bool MapSubscriber::hasBarrier(const geometry_msgs::Pose2D& pos)
{
    for (const json& receivedJson : lastReceivedJsons)
    {
        if (receivedJson["shape"]["type"] == "rect" && isInsideRectangle(pos, receivedJson))
            return true;
        else if (receivedJson["shape"]["type"] == "circle" && isInsideCircle(pos, receivedJson))
            return true;
    }
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
    lastReceivedJsons.clear();
    for (auto&& object : srv.response.objects)
    {
        lastReceivedJsons.push_back(json::parse(object));
        ROS_DEBUG_STREAM("Received from map: " << lastReceivedJsons.back().dump(4));
    }
}

bool Memory::MapSubscriber::isInsideRectangle(const geometry_msgs::Pose2D& pos, const nlohmann::json jsonRect) const
{
    double x, y, w, h;
    x = jsonRect["position"]["x"];
    y = jsonRect["position"]["y"];
    w = jsonRect["shape"]["width"];
    h = jsonRect["shape"]["height"];
    
    if (pos.x + (w/2) < x || pos.x - (w/2) > x)
        return false;
    if (pos.y + (h/2) < y || pos.y - (h/2) > y)
        return false;
    return true;
}

bool Memory::MapSubscriber::isInsideCircle(const geometry_msgs::Pose2D& pos, const nlohmann::json jsonCircle) const
{
    double x, y, r;
    x = jsonCircle["position"]["x"];
    y = jsonCircle["position"]["y"];
    r = jsonCircle["shape"]["radius"];
    
    if (getNorme2Distance(pos.x, pos.y, x, y) > r)
        return false;
    return true;
}

