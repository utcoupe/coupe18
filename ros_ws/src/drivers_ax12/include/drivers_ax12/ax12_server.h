
#ifndef DRIVERS_AX12_AX12_SERVER_H
#define DRIVERS_AX12_AX12_SERVER_H

#include <string>
#include <map>

#include <ros/console.h>
#include <actionlib/server/action_server.h>
#include <drivers_ax12/AngleCommandAction.h>
#include <drivers_port_finder/GetPort.h>
#include <memory_definitions/GetDefinition.h>

typedef actionlib::ServerGoalHandle<drivers_ax12::AngleCommandAction> GoalHandle;


class Ax12Server
{

protected:
    ros::NodeHandle nh_;
    actionlib::ActionServer<drivers_ax12::AngleCommandAction> as_;

    std::string action_name_, port_, def_file_path_;

    std::map<std::string, GoalHandle> current_goals; //maps id to goal

    // create messages that are used to published feedback/result
    drivers_ax12::AngleCommandFeedback feedback_;
    drivers_ax12::AngleCommandResult result_;


public:
    void execute_goal_cb(GoalHandle goal_handle);
    std::string fetch_port();
    std::string fetch_def_file_path();
    void set_ros_params();


    Ax12Server(std::string name) :
            as_(nh_, name, boost::bind(&Ax12Server::execute_goal_cb, this, _1), false),
            action_name_(name)
    {
        as_.start();

        def_file_path_ = fetch_def_file_path();
        // port_ = fetch_port();
        set_ros_params();



        ROS_INFO("drivers_ax12 action server initialized (port %s), waiting for goals...", port_.c_str());
    }




};

#endif //DRIVERS_AX12_AX12_SERVER_H
