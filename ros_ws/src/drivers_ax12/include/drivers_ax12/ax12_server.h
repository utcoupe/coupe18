
#ifndef DRIVERS_AX12_AX12_SERVER_H
#define DRIVERS_AX12_AX12_SERVER_H

#include <string>
#include <list>
#include <thread>

#include <ros/console.h>
#include <actionlib/server/action_server.h>
#include <drivers_ax12/AngleCommandAction.h>
#include <drivers_port_finder/GetPort.h>
#include <memory_definitions/GetDefinition.h>
#include <dynamixel_workbench.h>

const double MAX_STOP_TIME = 5; //number of seconds to wait not moving before confirming the goal is not reached
const double MAIN_FREQUENCY = 30;
const std::string PORT_FINDER_SERVICE = "/drivers/port_finder/get_port";
const std::string DEFAULT_PORT = "/dev/ttyACM0";
const uint32_t BAUD_RATE = 10000000;
const uint8_t SCAN_RANGE = 20;


typedef actionlib::ServerGoalHandle<drivers_ax12::AngleCommandAction> GoalHandle;


class Ax12Server
{

protected:
    ros::NodeHandle nh_;
    actionlib::ActionServer<drivers_ax12::AngleCommandAction> as_;

    DynamixelWorkbench dxl_wb_; // workbench to communicate with motors

    uint8_t dxl_id_[SCAN_RANGE]; // ids of all the motors connected
    uint8_t dxl_cnt_; // number of ids in above array

    std::list<GoalHandle> joint_goals;

    // create messages that are used to published feedback/result
    drivers_ax12::AngleCommandFeedback feedback_;
    drivers_ax12::AngleCommandResult result_;


public:
    void execute_goal_cb(GoalHandle goal_handle);
    std::string fetch_port(const std::string& service_name);
    void init_workbench(const std::string& port);
    bool motor_id_exists(uint8_t motor_id);
    bool position_is_valid(uint8_t motor_id, uint16_t position);
    void main_loop(const ros::TimerEvent&);
    bool handle_joint_goal(GoalHandle goal_handle);
    bool handle_wheel_goal(GoalHandle goal_handle);
    Ax12Server(std::string name);
    ~Ax12Server();




};

#endif //DRIVERS_AX12_AX12_SERVER_H
