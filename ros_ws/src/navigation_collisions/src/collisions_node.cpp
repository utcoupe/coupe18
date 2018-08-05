#include "ros/ros.h"
#include "navigation_collisions/PredictedCollision.h"
#include "navigation_collisions/ActivateCollisions.h"

#include "collisions_subscriptions.h"
#include "obstacles_stack.h"
#include "collisions_engine/engine.h"

#include "geometry_msgs/Pose2D.h"
#include <ai_game_status/init_service.h>

#include <sstream>

bool active = false;

ros::ServiceServer srv_activate;
ros::Publisher warner;

bool on_set_active(navigation_collisions::ActivateCollisions::Request &req,
                   navigation_collisions::ActivateCollisions::Response &res)
{
    active = req.active;
    if(active) ROS_INFO("Starting collisions check...");
    else ROS_INFO("Stopping collisions check.");
    res.success = true;
    return true;
}

void publish_collision(Collision collision) {
    navigation_collisions::PredictedCollision msg;

    switch(collision.level) {
        case CollisionLevel::LEVEL_STOP:
            msg.danger_level = msg.LEVEL_STOP;
            ROS_WARN("[COLLISION] Found freaking close collision, please stop !!");
            break;
        case CollisionLevel::LEVEL_DANGER:
            msg.danger_level = msg.LEVEL_DANGER;
            ROS_WARN("[COLLISION] Found close collision intersecting with the path.");
            break;
        case CollisionLevel::LEVEL_POTENTIAL:
            msg.danger_level = msg.LEVEL_POTENTIAL;
            ROS_INFO("[COLLISION] Found far-off collision intersecting with the path.");
            break;
    }

    MapObstacle & obs = collision.obstacle;
    msg.obstacle_pos.x = obs.position.x;
    msg.obstacle_pos.y = obs.position.y;
    msg.obstacle_pos.theta = obs.position.a;

    if(obs.type == ObstacleType::RECT)
    {
        RectObstacle & rect = static_cast<RectObstacle&>(obs);
        msg.obstacle_type = msg.TYPE_RECT;
        msg.obstacle_width = rect.width;
        msg.obstacle_height = rect.height;
    }
    else if(obs.type == ObstacleType::CIRCLE)
    {
        CircleObstacle & circle = static_cast<CircleObstacle&>(obs);
        msg.obstacle_type = msg.TYPE_CIRCLE;
        msg.obstacle_radius = circle.radius;
    }
    warner.publish(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "collisions");
    ros::NodeHandle n;

    // auto subscriptions = CollisionsSubscriptions(n);
    //Map::robot = subscriptions.create_robot();
    //auto markers = MarkersPublisher();

    srv_activate = n.advertiseService("/navigation/collisions/set_active", on_set_active);
    warner = n.advertise<navigation_collisions::PredictedCollision>("/navigation/collisions/warner", 10);
    StatusServices("navigation", "collisions").setReady(true);

    ros::Rate loop_rate(20);
    while (ros::ok())
    {
        // subscriptions.update_robot();
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