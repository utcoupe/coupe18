#ifndef COLLISIONS_SUBSCRIPTIONS_H
#define COLLISIONS_SUBSCRIPTIONS_H

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <vector>

#include "collisions_robot.h"
#include "collisions_engine/engine_shapes_attrib.h"


class CollisionsSubscriptions
{
public:
    CollisionsSubscriptions(ros::NodeHandle &n);
    ~CollisionsSubscriptions();

    void send_init(bool success = true);
    Robot create_robot();
    void update_robot();

protected:
    ros::NodeHandle &n;

    ros::Subscriber _sub_nav_status;
    ros::Subscriber _sub_classifier;
    ros::Subscriber _sub_robot_speed;

    NavStatus _nav_status = NavStatus::STATUS_IDLE;
    std::vector<Position> _robot_path_waypoints;
    float _vel_linear, _vel_angular;

    tf2_ros::Buffer _tf_buffer;
    tf2_ros::TransformListener _tl;

    Position _update_robot_pos();

    void _on_nav_status();
    void _on_classifier();
    void _on_robot_speed();

    std::vector<float> _quaternion_to_euler(std::vector<float> quaternion);
};


#endif