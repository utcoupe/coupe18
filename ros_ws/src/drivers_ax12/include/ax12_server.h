
#ifndef DRIVERS_AX12_AX12_SERVER_H
#define DRIVERS_AX12_AX12_SERVER_H

#include <string>
#include <list>
#include <thread>

#include <ros/console.h>
#include <actionlib/server/action_server.h>
#include <drivers_ax12/Ax12CommandAction.h>
#include <drivers_port_finder/GetPort.h>
#include <memory_definitions/GetDefinition.h>

#include "ax12_driver.h"


const double MAX_STOP_TIME = 5; //number of seconds to wait not moving before confirming the goal is not reached
const double MAIN_FREQUENCY = 30;
const std::string PORT_FINDER_SERVICE = "/drivers/port_finder/get_port";
const std::string DEFAULT_PORT = "/dev/ttyACM0";


typedef actionlib::ServerGoalHandle<drivers_ax12::Ax12CommandAction> GoalHandle;


class Ax12Server
{

protected:
    ros::NodeHandle nh_;
    actionlib::ActionServer<drivers_ax12::Ax12CommandAction> as_;

    std::list<GoalHandle> joint_goals_;

    // create messages that are used to published feedback/result
    drivers_ax12::Ax12CommandFeedback feedback_;
    drivers_ax12::Ax12CommandResult result_;

    Ax12Driver driver_;

public:
    void execute_goal_cb(GoalHandle goal_handle);
    std::string fetch_port(const std::string& service_name);
    void init_driver(const std::string& port);
    void main_loop(const ros::TimerEvent&);
    bool handle_joint_goal(GoalHandle goal_handle);
    bool handle_wheel_goal(GoalHandle goal_handle);
    Ax12Server(std::string name);
    ~Ax12Server();
};

#endif //DRIVERS_AX12_AX12_SERVER_H
