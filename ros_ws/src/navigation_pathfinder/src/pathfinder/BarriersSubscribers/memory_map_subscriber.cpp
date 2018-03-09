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
    return _occupancyGrid[pos.y][pos.x];
}

void MapSubscriber::subscribe(ros::NodeHandle& nodeHandle, [[maybe_unused]] std::size_t sizeMaxQueue, std::string topic)
{
    _srvGetMapObjects = nodeHandle.serviceClient<memory_map::MapGetObjects>(topic);
}

void MapSubscriber::fetchOccupancyData(const uint& widthGrid, const uint& heightGrid)
{
    memory_map::MapGetObjects srv;
    srv.request.collisions_only = true;
    if (!_srvGetMapObjects.call(srv) || !srv.response.success)
    {
        ROS_ERROR("Error when trying to call memory_map/MapGetObjects");
        return;
    }
    _lastReceivedJsons.clear();
    
    if (_occupancyGrid.size() != heightGrid || (heightGrid != 0 && _occupancyGrid.front().size() != widthGrid))
        _occupancyGrid = vector< vector<bool> > (
            heightGrid,
            vector<bool>(widthGrid, false)
        );
    
    for (auto&& object : srv.response.objects)
    {
        _lastReceivedJsons.push_back(json::parse(object));
        ROS_DEBUG_STREAM("Received from map: " << _lastReceivedJsons.back().dump(4));
    }
}

void Memory::MapSubscriber::drawRectangle(const nlohmann::json jsonRect)
{
    double x, y, w, h;
    x = jsonRect["position"]["x"];
    y = jsonRect["position"]["y"];
    w = jsonRect["shape"]["width"];
    h = jsonRect["shape"]["height"];
    
    auto pos = _convertor->fromRosToMapPos(make_pair(x, y));
    // TODO convert distances
    
    uint yMin = max(pos.second - (h/2) - _safetyMargin, 0.0);
    uint yMax = min((double)_occupancyGrid.size(), pos.second + (h/2) + _safetyMargin + 0.5);
    uint xMin = max(pos.first - (w/2) - _safetyMargin, 0.0);
    uint xMax = min((double)_occupancyGrid.front().size(), pos.first + (w/2) + _safetyMargin + 0.5);
    
    for (uint row = yMin; row < yMax; row++)
        for (uint column = xMin; column < xMax; column++)
            _occupancyGrid[row][column] = true;
}

void Memory::MapSubscriber::drawCircle(const nlohmann::json jsonCircle)
{
    double x, y, r;
    x = jsonCircle["position"]["x"];
    y = jsonCircle["position"]["y"];
    r = jsonCircle["shape"]["radius"];
    
    auto pos = _convertor->fromRosToMapPos(make_pair(x, y));
    
    uint yMin = max(y - r - _safetyMargin, 0.0);
    uint yMax = min((double)_occupancyGrid.size(), y + r + _safetyMargin + 0.5);
    uint xMin = max(x - r - _safetyMargin, 0.0);
    uint xMax = min((double)_occupancyGrid.front().size(), x + r + _safetyMargin + 0.5);
    
    for (uint row = yMin; row < yMax; row++)
        for (uint column = xMin; column < xMax; column++)
            if (getNorme2Distance(column, row, x, y) <= r)
                _occupancyGrid[row][column] = true;
}

