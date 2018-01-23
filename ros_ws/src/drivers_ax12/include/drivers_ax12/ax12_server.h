
#ifndef DRIVERS_AX12_AX12_SERVER_H
#define DRIVERS_AX12_AX12_SERVER_H

#include <string>
#include <map>

#include <ros/console.h>
#include <actionlib/server/action_server.h>
#include <drivers_ax12/AngleCommandAction.h>
#include <drivers_port_finder/GetPort.h>
#include <memory_definitions/GetDefinition.h>
#include <dynamixel_workbench.h>

const std::string DEFINITIONS_SERVICE = "/memory/definitions/get";
const std::string PORT_FINDER_SERVICE = "/drivers/port_finder/get_port";
const uint32_t BAUD_RATE = 10000000;
const uint8_t SCAN_RANGE = 20;
const uint32_t DEFAULT_VEL = 200; //1-1023 (if 0 then set to maximum) (0.111 rpm per unit)



typedef actionlib::ServerGoalHandle<drivers_ax12::AngleCommandAction> GoalHandle;


class Ax12Server
{

protected:
    ros::NodeHandle nh_;
    actionlib::ActionServer<drivers_ax12::AngleCommandAction> as_;

    DynamixelWorkbench dxl_wb_; // workbench to communicate with motors

    uint8_t dxl_id_[SCAN_RANGE]; // ids of all the motors connected
    uint8_t dxl_cnt_; // number of ids in above array

    std::map<std::string, GoalHandle> current_goals; //maps action id to goal

    // create messages that are used to published feedback/result
    drivers_ax12::AngleCommandFeedback feedback_;
    drivers_ax12::AngleCommandResult result_;


public:
    void execute_goal_cb(GoalHandle goal_handle);
    std::string fetch_port(const std::string& service_name);
    std::string fetch_def_file_path(const std::string& service_name);
    void set_ros_params(const std::string& def_file_path);
    void init_workbench(const std::string& port);


    Ax12Server(std::string name);
    ~Ax12Server();




};

#endif //DRIVERS_AX12_AX12_SERVER_H
