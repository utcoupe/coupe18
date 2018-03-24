#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include "navigation_pathfinder/FindPath.h"
#include "navigation_pathfinder/PathfinderNodeConfig.h"

#include "ai_game_status/init_service.h"

#include "pathfinder/map_storage.h"
#include "pathfinder/pathfinder.h"
#include "pathfinder/point.h"
#include "pathfinder/BarriersSubscribers/processing_belt_interpreter_subscriber.h"
#include "pathfinder/BarriersSubscribers/processing_lidar_objects_subscriber.h"
#include "pathfinder/BarriersSubscribers/memory_map_subscriber.h"
#include "pathfinder/BarriersSubscribers/recognition_objects_classifier_subscriber.h"
#include "pathfinder/pos_convertor.h"

#include <memory>

using namespace std;
using namespace Processing;
using namespace Memory;

const string                NAMESPACE_NAME          = "navigation";
const string                NODE_NAME               = "pathfinder";

const string                FINDPATH_SERVICE_NAME   = "/" + NAMESPACE_NAME + "/" + NODE_NAME + "/find_path";
const pair<double, double>  TABLE_SIZE              = {3.0, 2.0}; // Scale corresponding to messages received by the node
const string                MAP_FILE_NAME           = string(getenv ("UTCOUPE_WORKSPACE")) + "/ros_ws/src/memory_map/src/occupancy/img/layer_pathfinder.bmp"; //"/ros_ws/src/navigation_pathfinder/def/map.bmp";

const size_t                SIZE_MAX_QUEUE          = 10;
const double                SAFETY_MARGIN           = 0.15;
const string                BELT_INTERPRETER_TOPIC  = "/processing/belt_interpreter/rects";
const string                LIDAR_OBJECTS_TOPIC     = "/processing/lidar_objects/obstacles";
const string                MAP_GET_OBJECTS_SERVER  = "/memory/map/get_objects";

template<typename T>
unique_ptr<T> constructSubscriber(ros::NodeHandle& nodeHandle, const string& topic);

int main (int argc, char* argv[])
{
    ros::init(argc, argv, "pathfinder_node");
    
    ROS_INFO_STREAM("Starting pathfinder with map \"" + MAP_FILE_NAME + "\"...");
    
//     ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    ros::NodeHandle nodeHandle;
    
    auto convertor = make_shared<PosConvertor>();
    convertor->setInvertedY(true);
    
    auto dynBarriersMng = make_shared<DynamicBarriersManager>();
//     dynBarriersMng->addBarrierSubscriber(constructSubscriber<BeltInterpreterSubscriber>(nodeHandle, BELT_INTERPRETER_TOPIC));
//     dynBarriersMng->addBarrierSubscriber(constructSubscriber<LidarObjectsSubscriber>(nodeHandle, LIDAR_OBJECTS_TOPIC));
    auto mapSubscriber = constructSubscriber<MapSubscriber>(nodeHandle, MAP_GET_OBJECTS_SERVER);
    mapSubscriber->setConvertor(convertor);
    dynBarriersMng->addBarrierSubscriber(std::move(mapSubscriber));
    
    Pathfinder pathfinder(MAP_FILE_NAME, convertor, TABLE_SIZE, dynBarriersMng);
    ros::ServiceServer findPathServer = nodeHandle.advertiseService(FINDPATH_SERVICE_NAME, &Pathfinder::findPathCallback, &pathfinder);
    
    dynamic_reconfigure::Server<navigation_pathfinder::PathfinderNodeConfig> server;
    dynamic_reconfigure::Server<navigation_pathfinder::PathfinderNodeConfig>::CallbackType f;
    
    f = boost::bind(&Pathfinder::reconfigureCallback, &pathfinder, _1, _2);
    server.setCallback(f);
    
    StatusServices gameStatusSrv(NAMESPACE_NAME, NODE_NAME);
    gameStatusSrv.setReady(true);
    
    ros::spin();
    
    return 0;
}

template<typename T>
unique_ptr<T> constructSubscriber(ros::NodeHandle& nodeHandle, const string& topic)
{
    unique_ptr<T> subscriber(new T(SAFETY_MARGIN));
    subscriber->subscribe(nodeHandle, SIZE_MAX_QUEUE, topic);
    return subscriber;
}

