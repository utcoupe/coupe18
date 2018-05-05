#include <ros/ros.h>
#include <ros/console.h>
#include <dynamic_reconfigure/server.h>
#include <nlohmann/json.hpp>

#include "navigation_pathfinder/FindPath.h"
#include "navigation_pathfinder/PathfinderNodeConfig.h"

#include "ai_game_status/init_service.h"
#include "memory_map/MapGet.h"

#include "pathfinder/map_storage.h"
#include "pathfinder/pathfinder.h"
#include "pathfinder/point.h"
#include "pathfinder/BarriersSubscribers/memory_map_subscriber.h"
#include "pathfinder/BarriersSubscribers/recognition_objects_classifier_subscriber.h"
#include "pathfinder/pos_convertor.h"

#include <cmath>
#include <memory>

using namespace std;
using namespace Memory;
using namespace Recognition;

using json=nlohmann::json;

const string                NAMESPACE_NAME              = "navigation";
const string                NODE_NAME                   = "pathfinder";

const string                FINDPATH_SERVICE_NAME       = "/" + NAMESPACE_NAME + "/" + NODE_NAME + "/find_path";
const pair<double, double>  TABLE_SIZE                  = {3.0, 2.0}; // Scale corresponding to messages received by the node
const string                MAP_FILE_NAME               = string(getenv ("UTCOUPE_WORKSPACE")) + "/ros_ws/src/memory_map/src/occupancy/img/layer_pathfinder.bmp"; //"/ros_ws/src/navigation_pathfinder/def/map.bmp";

const size_t                SIZE_MAX_QUEUE              = 10;
const double                SAFETY_MARGIN               = 0.15;
const string                MAP_GET_OBJECTS_SERVER      = "/memory/map/get_objects";
const string                OBJECTS_CLASSIFIER_TOPIC    = "/recognition/objects_classifier/objects";

const string                MAP_GET_SERVER       = "/memory/map/get";

template<typename T>
unique_ptr<T> constructSubscriber(ros::NodeHandle& nodeHandle, const string& topic, double safety_margin);

double getDefaultMargin(ros::NodeHandle& nodeHandle);

int main (int argc, char* argv[])
{
    ros::init(argc, argv, "pathfinder_node");
    
    ROS_INFO_STREAM("Starting pathfinder with map \"" + MAP_FILE_NAME + "\"...");
    
//     ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    ros::NodeHandle nodeHandle;
    
    auto convertor = make_shared<PosConvertor>();
    convertor->setInvertedY(true);
    
    double safety_margin = getDefaultMargin(nodeHandle);
    ROS_INFO_STREAM("Pathfinder starting with safety_margin=" << safety_margin);
    
    auto dynBarriersMng = make_shared<DynamicBarriersManager>();
    auto mapSubscriber = constructSubscriber<MapSubscriber>(nodeHandle, MAP_GET_OBJECTS_SERVER, safety_margin);
    mapSubscriber->setConvertor(convertor);
    dynBarriersMng->addBarrierSubscriber(std::move(mapSubscriber));
    dynBarriersMng->addBarrierSubscriber(constructSubscriber<ObjectsClassifierSubscriber>(nodeHandle, OBJECTS_CLASSIFIER_TOPIC, safety_margin));
    
    Pathfinder pathfinder(MAP_FILE_NAME, convertor, TABLE_SIZE, dynBarriersMng);
    ros::ServiceServer findPathServer = nodeHandle.advertiseService(FINDPATH_SERVICE_NAME, &Pathfinder::findPathCallback, &pathfinder);
    
    dynamic_reconfigure::Server<navigation_pathfinder::PathfinderNodeConfig> server;
    dynamic_reconfigure::Server<navigation_pathfinder::PathfinderNodeConfig>::CallbackType f;
    
    f = boost::bind(&Pathfinder::reconfigureCallback, &pathfinder, _1, _2);
    server.setCallback(f);
    
    navigation_pathfinder::PathfinderNodeConfig config;
    server.getConfigDefault(config);
    config.safetyMargin = safety_margin;
    server.updateConfig(config);
    
    StatusServices gameStatusSrv(NAMESPACE_NAME, NODE_NAME);
    gameStatusSrv.setReady(true);
    
    ros::spin();
    
    return 0;
}

template<typename T>
unique_ptr<T> constructSubscriber(ros::NodeHandle& nodeHandle, const string& topic, double safety_margin)
{
    unique_ptr<T> subscriber(new T(safety_margin));
    subscriber->subscribe(nodeHandle, SIZE_MAX_QUEUE, topic);
    return subscriber;
}

double getDefaultMargin(ros::NodeHandle& nodeHandle)
{
    string robot_type;
    if (!nodeHandle.getParam("/robot", robot_type))
    {
        ROS_WARN("Error when trying to get '/robot' param. Falling back to default safety margin.");
        return SAFETY_MARGIN;
    }
    try
    {
        auto mapGetClient = nodeHandle.serviceClient<memory_map::MapGet>(MAP_GET_SERVER);
        memory_map::MapGet msg;
        msg.request.request_path = "/entities/" + robot_type + "/shape/*";
        mapGetClient.waitForExistence();
        if (!mapGetClient.call(msg) || !msg.response.success)
            throw ros::Exception("Call failed.");
        json shape = json::parse(msg.response.response);
        if (shape["type"] != "rect")
            throw ros::Exception("Shape '" + shape.at("type").get<string>() + "' not allowed.");
        double w, h;
        w = shape["width"];
        h = shape["height"];
        return sqrt(w*w/4 + h*h); // should be furthest point from robot even if its asserv center isn't its geometry center.
    }
    catch(ros::Exception e)
    {
        ROS_WARN_STREAM("Error when trying to contact '" + MAP_GET_SERVER + "' : " + e.what() + " Falling back to default safety margin.");
    }
    catch(...)
    {
        ROS_WARN_STREAM("Unknown error when trying to contact '" + MAP_GET_SERVER + "'. Falling back to default safety margin.");
    }
    return SAFETY_MARGIN;
}
