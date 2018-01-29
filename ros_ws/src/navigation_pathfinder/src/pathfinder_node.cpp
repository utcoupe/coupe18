#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include "navigation_pathfinder/FindPath.h"
#include "navigation_pathfinder/PathfinderNodeConfig.h"

#include "pathfinder/map_storage.h"
#include "pathfinder/pathfinder.h"
#include "pathfinder/point.h"
#include "pathfinder/BarriersSubscribers/processing_belt_interpreter_subscriber.h"

#include <cstdlib>
#include <iostream>
#include <vector>
#include <memory>

using namespace std;
using namespace Processing;

const string                FINDPATH_SERVICE_NAME   = "/navigation/pathfinder/find_path";
const pair<double, double>  TABLE_SIZE              = {3.0, 2.0}; // Scale corresponding to messages received by the node
const string                MAP_FILE_NAME           = string(getenv ("UTCOUPE_WORKSPACE")) + "/ros_ws/src/navigation_pathfinder/def/map.bmp";

const size_t                SIZE_MAX_QUEUE          = 10;
const string                BELT_INTERPRETER_TOPIC  = "/processing/belt_interpreter/rects_filtered";

unique_ptr<BeltInterpreterSubscriber> constructBeltInterpreterSubscriber(ros::NodeHandle& nodeHandle);

int main (int argc, char* argv[])
{
    ros::init(argc, argv, "pathfinder_node");
    
    ROS_INFO_STREAM("Starting pathfinder with map \"" + MAP_FILE_NAME + "\"...");
    
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    ros::NodeHandle nodeHandle;
    
    auto dynBarriersMng = make_shared<DynamicBarriersManager>();
    dynBarriersMng->addBarrierSubscriber(constructBeltInterpreterSubscriber(nodeHandle));
    
    Pathfinder pathfinder(MAP_FILE_NAME, TABLE_SIZE, dynBarriersMng, true);
    ros::ServiceServer findPathServer = nodeHandle.advertiseService(FINDPATH_SERVICE_NAME, &Pathfinder::findPathCallback, &pathfinder);
    
    dynamic_reconfigure::Server<navigation_pathfinder::PathfinderNodeConfig> server;
    dynamic_reconfigure::Server<navigation_pathfinder::PathfinderNodeConfig>::CallbackType f;
    
    f = boost::bind(&Pathfinder::reconfigureCallback, &pathfinder, _1, _2);
    server.setCallback(f);
    
    ROS_INFO("Pathfinder ready to find paths!");
    
    ros::spin();
    
    return 0;
}

unique_ptr<BeltInterpreterSubscriber> constructBeltInterpreterSubscriber(ros::NodeHandle& nodeHandle)
{
    unique_ptr<BeltInterpreterSubscriber> subscriber(new BeltInterpreterSubscriber());
    subscriber->subscribe(nodeHandle, SIZE_MAX_QUEUE, BELT_INTERPRETER_TOPIC);
    return subscriber;
}
