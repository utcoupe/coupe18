#include <cstdlib>
#include <iostream>
#include <vector>

#include <ros/ros.h>

#include "navigation_pathfinder/FindPath.h"

#include "pathfinder/map_storage.h"
#include "pathfinder/pathfinder.h"
#include "pathfinder/point.h"

using namespace std;

const string                FINDPATH_SERVICE_NAME   = "/navigation/pathfinder/find_path";
const pair<double, double>  TABLE_SIZE              = {3.0, 2.0}; // Scale corresponding to messages received by the node
const string                MAP_FILE_NAME           = string(getenv ("UTCOUPE_WORKSPACE")) + "/ros_ws/src/navigation_pathfinder/def/map.bmp";

int main (int argc, char* argv[])
{
    ros::init(argc, argv, "pathfinder_node");
    
    ROS_INFO_STREAM("Starting pathfinder with map \"" + MAP_FILE_NAME + "\"...");
    
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    ros::NodeHandle nodeHandle;
    
    Pathfinder pathfinder(MAP_FILE_NAME, TABLE_SIZE, true, true);
    ros::ServiceServer findPathServer = nodeHandle.advertiseService(FINDPATH_SERVICE_NAME, &Pathfinder::findPathCallback, &pathfinder);
    
    //test();
    //test2();
    
    ROS_INFO("Pathfinder ready to find paths!");
    
    ros::spin();
    
    return 0;
}
