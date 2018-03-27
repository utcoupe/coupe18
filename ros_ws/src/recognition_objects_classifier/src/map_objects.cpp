
#include "map_objects.h"
#include "memory_map/MapGet.h"
#include "ros/ros.h"
#include "nlohmann/json.hpp"

using namespace nlohmann;

void MapObjects::fetch_map_objects()
{
    ros::ServiceClient client = nh_.serviceClient<memory_map::MapGet>(MAP_GET_SERVICE);

    client.waitForExistence();

    memory_map::MapGet srv;

    srv.request.request_path = MAP_OBJECTS;

    if(client.call(srv) && srv.response.success)
    {
        auto objects = json::parse(srv.response.response);

        map_shapes_.clear();
        for(auto it = objects.begin(); it != objects.end(); it++)
        {

            if((*it)["position"]["frame_id"] != "/map")
            {
                ROS_ERROR("Map object not in /map !");
                continue;
            }


            std::string type = (*it)["shape"]["type"];

            float x = (*it)["position"]["x"];
            float y = (*it)["position"]["y"];

            if(type == "rect")
            {
                float height = (*it)["shape"]["height"];
                float width = (*it)["shape"]["width"];

                map_shapes_.push_back(std::make_shared<const Rectangle>(x, y, width, height));

            }
            else if(type == "circle")
            {
                float radius = (*it)["shape"]["radius"];

                map_shapes_.push_back(std::make_shared<const Circle>(x, y, radius));
            }
            else
            {
                ROS_ERROR("Polygons from map not supported !");
            }

        }

        ROS_INFO("Fetched %d map shapes successfully", (int)map_shapes_.size());

    }
    else
    {
        ROS_ERROR("Failed to contact memory_map, static objects not fetched");
    }
}

bool MapObjects::contains_point(float x, float y)
{
    for(auto it = map_shapes_.begin(); it != map_shapes_.end(); it++)
    {
        if((*it)->contains_point(x, y))
            return true;
    }

    return false;
}