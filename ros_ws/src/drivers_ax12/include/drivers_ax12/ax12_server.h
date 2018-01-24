
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
const std::string DEFINITIONS_SERVICE = "/memory/definitions/get";
const std::string DEFINITION_PATH = "drivers/ax12.yaml";
const std::string PORT_FINDER_SERVICE = "/drivers/port_finder/get_port";
const std::string DEFAULT_PORT = "/dev/ttyACM0";
const uint32_t BAUD_RATE = 10000000;
const uint8_t SCAN_RANGE = 20;
const uint16_t DEFAULT_VEL = 200; //1-1023 (if 0 then set to maximum) (0.111 rpm per unit)
const uint16_t DEFAULT_MIN = 0; //0-1023
const uint16_t DEFAULT_MAX = 1023; //0-1023


typedef actionlib::ServerGoalHandle<drivers_ax12::AngleCommandAction> GoalHandle;


class Ax12Server
{

protected:
    ros::NodeHandle nh_;
    actionlib::ActionServer<drivers_ax12::AngleCommandAction> as_;

    DynamixelWorkbench dxl_wb_; // workbench to communicate with motors

    uint8_t dxl_id_[SCAN_RANGE]; // ids of all the motors connected
    uint8_t dxl_cnt_; // number of ids in above array

    std::list<GoalHandle> current_goals;

    // create messages that are used to published feedback/result
    drivers_ax12::AngleCommandFeedback feedback_;
    drivers_ax12::AngleCommandResult result_;


public:
    void execute_goal_cb(GoalHandle goal_handle);
    std::string fetch_port(const std::string& service_name);
    std::string fetch_def_file_path(const std::string& service_name);
    bool set_ros_params(const std::string& def_file_path);
    void init_workbench(const std::string& port);
    bool init_motor(uint8_t motor_id);
    bool motor_id_exists(uint8_t motor_id);
    bool position_is_valid(uint8_t motor_id, uint16_t position);
    void main_loop(const ros::TimerEvent&);

    Ax12Server(std::string name);
    ~Ax12Server();




};

#endif //DRIVERS_AX12_AX12_SERVER_H
