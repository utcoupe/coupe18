#include "ros/ros.h"
#include "navigation_coppisions/PredictedCollision.h"
#include "navigation_coppisions/ActivateCollisions.h"

#include "collisions_node.h"
#include "obstacles_stack.h"
#include "collisions_engine/engine.h"

#include "geometry_msgs/Pose2D.h"

#include <sstream>

bool active = false;

ros::ServiceServer srv_activate;
ros::Publisher warner;

bool on_set_active(navigation_coppisions::ActivateCollisions::Request &req,
                   navigation_coppisions::ActivateCollisions::Response &res)
{
    active = req.active;
    if(active) ROS_INFO("Starting collisions check...");
    else ROS_INFO("Stopping collisions check.");
    res.success = true;
    return true;
}

void publish_collision(Collision collision) {
    navigation_coppisions::PredictedCollision msg;
    msg.danger_level = collision.level;

    switch(collision.level) {
        case CollisionLevel::LEVEL_STOP:
            ROS_WARN("[COLLISION] Found freaking close collision, please stop !!");
            break;
        case CollisionLevel::LEVEL_DANGER:
            ROS_WARN("[COLLISION] Found close collision intersecting with the path.");
            break;
        case CollisionLevel::LEVEL_POTENTIAL:
            ROS_INFO("[COLLISION] Found far-off collision intersecting with the path.");
            break;
    }

    MapObstacle obs = collision.obstacle;
    msg.obstacle_pos = Pose2D(obs.position.x, obs.position.y, obs.position.a);
    if(obs.type == ObstacleType::RECT)
    {
        msg.obstacle_type = msg.TYPE_RECT;
        msg.obstacle_width = (RectObstacle)obs.width;
        msg.obstacle_height = (RectObstacle)obs.height;
    }
    else if(obs.type == ObstacleType::CIRCLE)
    {
        msg.obstacle_type = msg.TYPE_CIRCLE;
        msg.obstacle_radius = (CircleObstacle)obs.radius;
    }
    warner.publish(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "collisions");
    ros::NodeHandle n;

    // auto subscriptions = CollisionsSubscriptions();
    // Map::robot = subscriptions.create_robot();
    // auto markers = MarkersPublisher();

    srv_activate = n.advertiseService("/navigation/collisions/set_active", on_set_active);
    warner = n.advertise<navigation_coppisions::PredictedCollision>("/navigation/collisions/warner", 10);
    // subscripions.send_init();

    ros::Rate loop_rate(20);
    while (ros::ok())
    {
        //subscriptions.update_robot();
        if(active) {
            std::vector<Collision> collisions = Map::robot.check_collisions(ObstaclesStack::to_list());
            for(int i = 0; i < collisions.size(); i++)
                publish_collision(collisions[i]);
            //markers.publish_check_zones(Map.Robot);
        }

        //markers.publish_obstacles(ObstaclesStack.toList());
        ObstaclesStack::garbage_collect();

        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}