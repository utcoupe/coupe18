
#include "map_objects.h"
#include "memory_map/MapGet.h"
#include "ros/ros.h"

void MapObjects::fetch_map_objects()
{
    ros::ServiceClient client = nh_.serviceClient<memory_map::MapGet>(MAP_GET_SERVICE);

    client.waitForExistence();

    memory_map::MapGet srv;

    srv.request.request_path = MAP_OBJECTS;

    if(client.call(srv) && srv.response.success)
    {
        
    }
    else
    {
        ROS_ERROR("Failed to contact memory_map, static objects not fetched");
    }
}