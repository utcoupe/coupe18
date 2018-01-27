
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

// words
const int CW_ANGLE_LIMIT_ADDR = 6;
const int CCW_ANGLE_LIMIT_ADDR = 8;
const int GOAL_POSITION_ADDR = 30;
const int MOVING_SPEED_ADDR = 32;
const int PRESENT_POSITION_ADDR = 36;

// bytes
const int TORQUE_ENABLE_ADDR = 24;
const int MOVING_ADDR = 46;


const double MAX_STOP_TIME = 5; //number of seconds to wait not moving before confirming the goal is not reached
const double MAIN_FREQUENCY = 30;
const std::string PORT_FINDER_SERVICE = "/drivers/port_finder/get_port";
const std::string DEFAULT_PORT = "/dev/ttyACM0";
const uint32_t BAUD_RATE_INDEX = 1; //baudrate = 2000000 / (index + 1)
const uint8_t SCAN_RANGE = 20;


typedef actionlib::ServerGoalHandle<drivers_ax12::AngleCommandAction> GoalHandle;


class Ax12Server
{

protected:
    ros::NodeHandle nh_;
    actionlib::ActionServer<drivers_ax12::AngleCommandAction> as_;

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
