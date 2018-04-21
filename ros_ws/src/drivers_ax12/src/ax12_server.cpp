#include "ax12_server.h"

#include <ai_game_status/ArmRequest.h>

#include <memory>

using namespace Ax12Table;


void Ax12Server::init_driver(const std::string &port) {

    if (!driver_.initialize(port)) {
        ROS_FATAL("Unable to initialize the AX-12 SDK, shutting down !");
        ros::shutdown();
        return;
    }

    driver_.scan_motors();

    driver_.toggle_torque(true);
}

void Ax12Server::cancel_goal_cb(GoalHandle goal_handle) {
    uint8_t motor_id = goal_handle.getGoal()->motor_id;

    ROS_INFO("Received a cancel request, stopping movement of motor %d", motor_id);

    uint16_t moving;

    driver_.read_register(motor_id, MOVING, moving);

    if (!moving) {
        ROS_WARN("Received a cancel request but motor %d is not moving", motor_id);
        return;
    }

    uint16_t pos;
    driver_.joint_mode(motor_id);
    driver_.read_register(motor_id, PRESENT_POSITION, pos);
    driver_.write_register(motor_id, GOAL_POSITION, pos);

    for (auto it = joint_goals_.begin(); it != joint_goals_.end(); it++) {
        if (goal_handle.getGoalID().id == it->getGoalID().id) {
            joint_goals_.erase(it);
            ROS_DEBUG("Erased joint goal of motor %d from the list", motor_id);
            return;
        }
    }
}

void Ax12Server::execute_goal_cb(GoalHandle goal_handle) {
    /*
     * Handles the goal requests of the action server.
     * Checks if the goal is valid, then tells the motor
     * to move and adds the goal to the list of current goals
     */

    auto goal = goal_handle.getGoal();
    uint8_t motor_id = goal->motor_id;

    if (is_halted) {
        ROS_ERROR("AX-12 action server received a goal, but game_status said that the system is halted !");
        result_.success = 0;
        goal_handle.setRejected(result_);
        return;
    }

    if (!goal_handle.isValid()) {
        ROS_ERROR("AX-12 action server received an invalid goal !");
        result_.success = 0;
        goal_handle.setRejected(result_);
        return;
    }

    if (!driver_.motor_id_exists(motor_id)) {
        ROS_ERROR("AX-12 action server received a goal for motor ID %d, but no such motor was detected", motor_id);
        result_.success = 0;
        goal_handle.setRejected(result_);
        return;
    }

    if (!driver_.motor_id_connected(motor_id)) {
        ROS_ERROR("AX-12 action server received a goal for motor ID %d, but the motor was disconnected", motor_id);
        result_.success = 0;
        goal_handle.setRejected(result_);
        return;
    }

    if (goal->mode == goal->JOINT)
        handle_joint_goal(goal_handle);
    else
        handle_wheel_goal(goal_handle);
}

bool Ax12Server::handle_joint_goal(GoalHandle goal_handle) {
    auto goal = goal_handle.getGoal();
    uint8_t motor_id = goal->motor_id;
    uint16_t position = goal->position;

    if (position <= 0 || position > 1023) {
        ROS_ERROR("AX-12 action server received a joint goal for motor ID %d, but with an invalid position: %d",
                  motor_id, position);
        result_.success = 0;
        goal_handle.setRejected(result_);
        return false;
    }

    uint16_t speed = goal->speed;
    if (speed < 0 || speed > 1023) {
        ROS_ERROR("AX-12 action server received a joint goal for motor ID %d,"
                  " but with an invalid speed: %d", motor_id, speed);
        result_.success = 0;
        goal_handle.setRejected(result_);
        return false;
    }

    goal_handle.setAccepted();

    for (auto it = joint_goals_.begin(); it != joint_goals_.end();) {
        if (it->getGoal()->motor_id == motor_id) {
            it->setCanceled();
            ROS_WARN("AX-12 action server received a joint goal for motor ID %d"
                     " while another joint goal was running for that motor."
                     " The old goal was canceled.", motor_id);

            it = joint_goals_.erase(it);
        } else {
            it++;
        }
    }

    bool success = true;

    success &= driver_.joint_mode(motor_id);

    success &= driver_.write_register(motor_id, GOAL_POSITION, position);
    success &= driver_.write_register(motor_id, MOVING_SPEED, speed);

    joint_goals_.push_back(goal_handle);
    ROS_DEBUG("Success setting goal and speed for motor %d, adding the goal to the list", motor_id);

    return success;
}

bool Ax12Server::handle_wheel_goal(GoalHandle goal_handle) {
    auto goal = goal_handle.getGoal();
    uint8_t motor_id = goal->motor_id;
    uint16_t speed = goal->speed;

    if (speed < 0 || speed > 2047) {
        ROS_ERROR("AX-12 action server received a wheel goal for motor ID %d,"
                  " but with an invalid speed: %d", motor_id, speed);
        result_.success = 0;
        goal_handle.setRejected(result_);
        return false;
    }

    goal_handle.setAccepted();

    for (auto it = joint_goals_.begin(); it != joint_goals_.end();) {
        if (it->getGoal()->motor_id == motor_id) {
            result_.success = 0;
            it->setCanceled(result_);
            ROS_INFO("AX-12 action server received a wheel goal for motor ID %d while "
                     "another joint goal was running for that motor. "
                     "The old goal was canceled.", motor_id);
            it = joint_goals_.erase(it);
        } else {
            it++;
        }
    }

    bool success = true;
    success &= driver_.wheel_mode(motor_id);
    success &= driver_.write_register(motor_id, MOVING_SPEED, speed);


    result_.success = static_cast<unsigned char>(success);
    goal_handle.setSucceeded(result_);

    return success;
}

void Ax12Server::main_loop(const ros::TimerEvent &) {
    /*
     * Called on a Timer, checks the status of all current goals and updates them
     * Feedback is published too
     */

    uint8_t motor_id = 0;
    uint16_t curr_position = 0;
    uint32_t goal_position = 0;

    for (auto it = joint_goals_.begin(); it != joint_goals_.end();) {
        motor_id = it->getGoal()->motor_id;
        driver_.read_register(motor_id, PRESENT_POSITION, curr_position);

        ROS_DEBUG("Motor %d : position %d", motor_id, curr_position);

        goal_position = it->getGoal()->position;

        if (curr_position >= (goal_position - POSITION_MARGIN) && curr_position <= (goal_position + POSITION_MARGIN)) {
            ROS_DEBUG("Motor has reached the goal position ! curr_pos : %d, goal_pos : %d", curr_position,
                      goal_position);
            result_.success = 1;
            it->setSucceeded(result_);
            it = joint_goals_.erase(it);
            ROS_INFO("AX-12 position goal %d succeeded for motor ID %d", goal_position, motor_id);

        } else if (ros::Time::now().toSec() - it->getGoalID().stamp.toSec() > MAX_STOP_TIME) {
            ROS_ERROR("Motor has not reached the goal position, timeout reached ! curr_pos : %d, goal_pos : %d",
                      curr_position, goal_position);

            // reset the alarm
            driver_.write_register(motor_id, TORQUE_ENABLE, 0);
            driver_.write_register(motor_id, TORQUE_ENABLE, 1);

            result_.success = 0;
            it->setAborted(result_);
            it = joint_goals_.erase(it);
        } else {
            feedback_.position = curr_position;
            it->publishFeedback(feedback_);
            it++;
        }
    }
}

std::string Ax12Server::fetch_port(const std::string &service_name) {
    /*
     * Asks the port_finder for the motor port
     */

    std::string port;

    if (!ros::service::waitForService(service_name, 15000)) {
        ROS_ERROR("Failed to contact drivers_port_finder (service not up)");
    } else {
        ros::ServiceClient client = nh_.serviceClient<drivers_port_finder::GetPort>(service_name);
        drivers_port_finder::GetPort srv;
        srv.request.component = "usb2ax";

        if (client.call(srv)) {
            port = srv.response.port;
        } else {
            ROS_ERROR("Failed to fetch the AX-12 port from drivers_port_finder");
        }
    }

    if (port.length() == 0) {
        ROS_FATAL("The AX-12 port is not set, shutting down...");
        ros::shutdown();
        return "";
    }

    return port;
}

const Ax12Table::Register *Ax12Server::service_param_to_register(uint8_t param) {
    switch (param) {
        case drivers_ax12::SetAx12ParamRequest::ID:
            return &ID;
        case drivers_ax12::SetAx12ParamRequest::BAUD_RATE:
            return &BAUD_RATE;
        case drivers_ax12::SetAx12ParamRequest::RETURN_DELAY_TIME:
            return &RETURN_DELAY_TIME;
        case drivers_ax12::SetAx12ParamRequest::CW_ANGLE_LIMIT:
            return &CW_ANGLE_LIMIT;
        case drivers_ax12::SetAx12ParamRequest::CCW_ANGLE_LIMIT:
            return &CCW_ANGLE_LIMIT;
        case drivers_ax12::SetAx12ParamRequest::HIGHEST_TEMPERATURE:
            return &HIGHEST_TEMPERATURE;
        case drivers_ax12::SetAx12ParamRequest::LOWEST_VOLTAGE:
            return &LOWEST_VOLTAGE;
        case drivers_ax12::SetAx12ParamRequest::HIGHEST_VOLTAGE:
            return &HIGHEST_VOLTAGE;
        case drivers_ax12::SetAx12ParamRequest::MAX_TORQUE:
            return &MAX_TORQUE;
        case drivers_ax12::SetAx12ParamRequest::STATUS_RETURN_LEVEL:
            return &STATUS_RETURN_LEVEL;
        case drivers_ax12::SetAx12ParamRequest::ALARM_LED:
            return &ALARM_LED;
        case drivers_ax12::SetAx12ParamRequest::ALARM_SHUTDOWN:
            return &ALARM_SHUTDOWN;
        case drivers_ax12::SetAx12ParamRequest::TORQUE_ENABLE:
            return &TORQUE_ENABLE;
        case drivers_ax12::SetAx12ParamRequest::LED:
            return &LED;
        case drivers_ax12::SetAx12ParamRequest::CW_COMPLIANCE_MARGIN:
            return &CW_COMPLIANCE_MARGIN;
        case drivers_ax12::SetAx12ParamRequest::CCW_COMPLIANCE_MARGIN:
            return &CCW_COMPLIANCE_MARGIN;
        case drivers_ax12::SetAx12ParamRequest::CW_COMPLIANCE_SLOPE:
            return &CW_COMPLIANCE_SLOPE;
        case drivers_ax12::SetAx12ParamRequest::CCW_COMPLIANCE_SLOPE:
            return &CCW_COMPLIANCE_SLOPE;
        case drivers_ax12::SetAx12ParamRequest::GOAL_POSITION:
            return &GOAL_POSITION;
        case drivers_ax12::SetAx12ParamRequest::MOVING_SPEED:
            return &MOVING_SPEED;
        case drivers_ax12::SetAx12ParamRequest::TORQUE_LIMIT:
            return &TORQUE_LIMIT;
        case drivers_ax12::SetAx12ParamRequest::LOCK:
            return &LOCK;
        case drivers_ax12::SetAx12ParamRequest::PUNCH:
            return &PUNCH;
        default:
            return nullptr;
    }
}

bool Ax12Server::execute_set_service_cb(drivers_ax12::SetAx12Param::Request &req,
                                        drivers_ax12::SetAx12Param::Response &res) {

    if (!driver_.motor_id_exists(req.motor_id)) {
        ROS_ERROR("AX-12 set_param service server received a request for motor ID %d, but no such motor was detected",
                  req.motor_id);
        res.success = 0;
        return true;
    }

    if (!driver_.motor_id_connected(req.motor_id)) {
        ROS_ERROR("AX-12 set_param service server received a request for motor ID %d, but the motor was disconnected",
                  req.motor_id);
        res.success = 0;
        return true;
    }

    const Register *reg = service_param_to_register(req.param);

    res.success = (uint8_t) driver_.write_register(req.motor_id, *reg, static_cast<uint16_t>(req.value));

    ROS_INFO("Successfully changed parameter %d of motor with ID %d to %d", req.param, req.motor_id, req.value);

    return true;
}

void Ax12Server::game_status_cb(const ai_game_status::GameStatusConstPtr &status) {
    if (!is_halted && status->game_status == status->STATUS_HALT) {
        is_halted = true;
        driver_.toggle_torque(false);
    } else if (is_halted && status->game_status != status->STATUS_HALT) {
        is_halted = false;
        driver_.toggle_torque(true);
    }
}

Ax12Server::Ax12Server(const std::string &action_name, const std::string &service_name) :
        as_(nh_, action_name, boost::bind(&Ax12Server::execute_goal_cb, this, _1),
            boost::bind(&Ax12Server::cancel_goal_cb, this, _1), false),
        set_param_service(nh_.advertiseService(service_name, &Ax12Server::execute_set_service_cb, this)),
        game_status_sub_(nh_.subscribe(GAME_STATUS_TOPIC, 30, &Ax12Server::game_status_cb, this)),
        joint_goals_(),
        feedback_(),
        result_(),
        driver_(),
        is_halted(false) {

    as_.start();

    std::string port = fetch_port(PORT_FINDER_SERVICE);

    init_driver(port);

    ROS_INFO("AX-12 action server initialized for port %s, waiting for goals", port.c_str());

    timer_ = nh_.createTimer(ros::Duration(1.0 / MAIN_FREQUENCY), &Ax12Server::main_loop, this);

    status_services_ = std::make_unique<StatusServices>(
        "drivers", "ax12", [this, port](const ai_game_status::ArmRequest::ConstPtr &){
            this->_on_armRequest(port);
        });

    status_services_->setReady(true);
}

Ax12Server::~Ax12Server() {
    driver_.toggle_torque(false);
    ros::shutdown();
}

void Ax12Server::_on_armRequest(std::string port)
{
    driver_.scan_motors();
    driver_.toggle_torque(true);
}

