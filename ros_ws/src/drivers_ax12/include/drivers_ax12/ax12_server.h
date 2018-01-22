
#ifndef DRIVERS_AX12_AX12_SERVER_H
#define DRIVERS_AX12_AX12_SERVER_H

#include <string>
#include <map>

#include <ros/console.h>
#include <actionlib/server/action_server.h>
#include <drivers_ax12/AngleCommandAction.h>

typedef actionlib::ServerGoalHandle<drivers_ax12::AngleCommandAction> GoalHandle;


class Ax12Server
{

protected:
    ros::NodeHandle nh_;
    actionlib::ActionServer<drivers_ax12::AngleCommandAction> as_;

    std::string action_name_;

    std::map<std::string, GoalHandle> current_goals; //maps id to goal

    // create messages that are used to published feedback/result
    drivers_ax12::AngleCommandFeedback feedback_;
    drivers_ax12::AngleCommandResult result_;


public:
    Ax12Server(std::string name) :
            as_(nh_, name, boost::bind(&Ax12Server::execute_goal_cb, this, _1), false),
            action_name_(name)
    {
        as_.start();

        ROS_INFO("drivers_ax12 action server initialized, waiting for goals...");
    }

    void execute_goal_cb(GoalHandle goal_handle);


};

#endif //DRIVERS_AX12_AX12_SERVER_H
