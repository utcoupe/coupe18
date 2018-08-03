#include "ros/ros.h"
#include "navigation_coppisions/PredictedCollision.h"
#include "navigation_coppisions/ActivateCollisions.h"

#include <sstream>

bool active = false;

ros::ServiceServer srv_activate;
ros::Publisher pub;

bool on_set_active(navigation_coppisions::ActivateCollisions::Request &req,
                   navigation_coppisions::ActivateCollisions::Response &res)
{
    active = req.active;
    if(active) ROS_INFO("Starting collisions check...");
    else ROS_INFO("Stopping collisions check.");
    res.success = true;
    return true;
}

/*void publish_collision(Collision collision) {
    navigation_coppisions::PredictedCollision msg;
    msg.danger_level = collision.level;

    switch(collision.level) {
        case CollisionLevel.LEVEL_STOP:
            ROS_WARN("[COLLISION] Found freaking close collision, please stop !!");
            break;
        case CollisionLevel.LEVEL_DANGER:
            ROS_WARN("[COLLISION] Found close collision intersecting with the path.");
            break;
        case CollisionLevel.LEVEL_POTENTIAL:
            ROS_INFO("[COLLISION] Found far-off collision intersecting with the path.");
            break;
    }
    TODO
    obs = collision.obstacle
    m.obstacle_pos = Pose2D(obs.position.x, obs.position.y, obs.position.a)
    if isinstance(obs, RectObstacle):
        m.obstacle_type = m.TYPE_RECT
        m.obstacle_width, m.obstacle_height = obs.width, obs.height
    elif isinstance(obs, CircleObstacle):
        m.obstacle_type = m.TYPE_CIRCLE
        m.obstacle_radius = obs.radius
    
}*/

int main(int argc, char **argv)
{
    ros::init(argc, argv, "collisions");
    ros::NodeHandle n;

    // auto subscriptions = CollisionsSubscriptions();
    // Map.Robot = subscriptions.create_robot();
    // auto markers = MarkersPublisher();

    srv_activate = n.advertiseService("/navigation/collisions/set_active", on_set_active);
    pub = n.advertise<navigation_coppisions::PredictedCollision>("/navigation/collisions/warner", 10);
    // subscripions.send_init();

    ros::Rate loop_rate(20);
    while (ros::ok())
    {
        /*subscriptions.update_robot();
        if(active) {
            auto collisions = Map.Robot.check_collisions(ObstaclesStack.toList());
            for(int i = 0; i < collisions.size(); i++)
                publish_collision(collisions[i]);
            markers.publish_check_zones(Map.Robot);
        }

        markers.publish_obstacles(ObstaclesStack.toList());
        ObstaclesStack.garbage_collect();*/

        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}