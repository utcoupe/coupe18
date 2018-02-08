#include "ax12_server.h"

using namespace Ax12Table;


void Ax12Server::init_driver(const std::string& port)
{

    std::string param_key;
    int8_t port_index;

    // get the port index from the port string
    size_t port_num_index = port.find_last_of("0123456789");
    port_index = port[port_num_index] - '0';

    if(port_num_index == std::string::npos || port_index < 0 || port_index > 9)
        ROS_ERROR("Unable to get the port index from the port %s", port.c_str());

    if(!driver_.initialize(port_index))
    {
        ROS_FATAL("Unable to initialize the AX-12 driver with port index %d and baudrate index %d", port_index, driver_.BAUD_RATE_INDEX);
        ros::shutdown();
        return;
    }

    driver_.scan_motors();

    ROS_INFO("Found %d AX-12 motors connected with a scan range of %d", driver_.get_motor_count(), driver_.SCAN_RANGE);

    driver_.toggle_torque(true);

}

void Ax12Server::execute_goal_cb(GoalHandle goal_handle)
{
    /*
     * Handles the goal requests of the action server.
     * Checks if the goal is valid, then tells the motor
     * to move and adds the goal to the list of current goals
     */

    auto goal = goal_handle.getGoal();
    uint8_t motor_id = goal->motor_id;

    if(!goal_handle.isValid())
    {
        ROS_ERROR("AX-12 action server received an invalid goal !");
        goal_handle.setRejected();
        return;
    }

    if(!driver_.motor_id_exists(motor_id))
    {
        ROS_ERROR("AX-12 action server received a goal for motor ID %d, but no such motor was detected", motor_id);
        goal_handle.setRejected();
        return;
    }

    if(!driver_.motor_id_connected(motor_id))
    {
        ROS_ERROR("AX-12 action server received a goal for motor ID %d, but the motor was disconnected", motor_id);
        goal_handle.setRejected();
        return;
    }

    if(goal->mode == goal->JOINT)
        handle_joint_goal(goal_handle);
    else
        handle_wheel_goal(goal_handle);
}

bool Ax12Server::handle_joint_goal(GoalHandle goal_handle)
{
    auto goal = goal_handle.getGoal();
    uint8_t motor_id = goal->motor_id;
    uint16_t position = goal->position;

    if(position <= 0 || position > 1023)
    {
        ROS_ERROR("AX-12 action server received a joint goal for motor ID %d, but with an invalid position: %d", motor_id, position);
        goal_handle.setRejected();
        return false;
    }

    uint16_t speed = goal->speed;
    if(speed < 0 || speed > 1023)
    {
        ROS_ERROR("AX-12 action server received a joint goal for motor ID %d, but with an invalid speed: %d", motor_id, speed);
        goal_handle.setRejected();
        return false;
    }

    goal_handle.setAccepted();

    for(auto it = joint_goals_.begin(); it != joint_goals_.end();)
    {
        if(it->getGoal()->motor_id == motor_id)
        {
            it->setCanceled();
            ROS_INFO("AX-12 action server received a joint goal for motor ID %d while another joint goal was running for that motor. The old goal was canceled.", motor_id);
            it = joint_goals_.erase(it);
        }
        else
        {
            it++;
        }
    }

    driver_.joint_mode(motor_id);
    driver_.write_register(motor_id, MOVING_SPEED, speed);
    driver_.write_register(motor_id, GOAL_POSITION, position);


    joint_goals_.push_back(goal_handle);
    ROS_DEBUG("Success setting goal and speed, adding the goal to the list");
    return true;
}

bool Ax12Server::handle_wheel_goal(GoalHandle goal_handle)
{
    auto goal = goal_handle.getGoal();
    uint8_t motor_id = goal->motor_id;
    uint16_t speed = goal->speed;

    if(speed < 0 || speed > 2047)
    {
        ROS_ERROR("AX-12 action server received a wheel goal for motor ID %d, but with an invalid speed: %d", motor_id, speed);
        goal_handle.setRejected();
        return false;
    }

    goal_handle.setAccepted();

    for(auto it = joint_goals_.begin(); it != joint_goals_.end();)
    {
        if(it->getGoal()->motor_id == motor_id)
        {
            it->setCanceled();
            ROS_INFO("AX-12 action server received a wheel goal for motor ID %d while another joint goal was running for that motor. The old goal was canceled.", motor_id);
            it = joint_goals_.erase(it);
        }
        else
        {
            it++;
        }
    }

    driver_.wheel_mode(motor_id);
    driver_.write_register(motor_id, MOVING_SPEED, speed);


    result_.success = true;
    goal_handle.setSucceeded(result_);

    return true;
}

void Ax12Server::main_loop(const ros::TimerEvent&)
{
    /*
     * Called on a Timer, checks the status of all current goals and updates them
     * Feedback is published too
     */

    uint8_t motor_id;
    int16_t curr_position;
    int16_t moving;
    int32_t goal_position;

    ROS_DEBUG("Entered feedback loop");

    for(auto it = joint_goals_.begin(); it != joint_goals_.end();)
    {
        motor_id = it->getGoal()->motor_id;
        ROS_DEBUG("Checking state of motor %d", motor_id);

        driver_.read_register(motor_id, PRESENT_POSITION, curr_position);
        driver_.read_register(motor_id, MOVING, moving);
        if(moving)
        {
            ROS_DEBUG("Motor is moving, publishing feedback");
            feedback_.position = curr_position;
            it->publishFeedback(feedback_);
            it++;
            continue;
        }

        goal_position = it->getGoal()->position;

        if(curr_position >= (goal_position - POSITION_MARGIN) && curr_position <= (goal_position + POSITION_MARGIN))
        {
            ROS_DEBUG("Motor has reached the goal position");
            result_.success = true;
            it->setSucceeded(result_);
            it = joint_goals_.erase(it);
            ROS_INFO("AX-12 position goal succeeded for motor ID %d", motor_id);

        }
        else if(ros::Time::now().toSec() - it->getGoalID().stamp.toSec() > MAX_STOP_TIME)
        {
            ROS_DEBUG("Timeout reached for motor");
            result_.success = false;
            it->setAborted(result_);
            it = joint_goals_.erase(it);
            ROS_ERROR("AX-12 position goal aborted for motor ID %d", motor_id);
        }
        else
        {
            ROS_DEBUG("Motor stopped but still no timeout, publishing feedback");
            feedback_.position = curr_position;
            it->publishFeedback(feedback_);
            it++;
        }

    }

}



std::string Ax12Server::fetch_port(const std::string& service_name)
{
    /*
     * Asks the port_finder for the motor port
     */

    std::string port;

    if(!ros::service::waitForService(service_name, 15000))
    {
        ROS_ERROR("Failed to contact drivers_port_finder (service not up)");
    }
    else
    {
        ros::ServiceClient client = nh_.serviceClient<drivers_port_finder::GetPort>(service_name);
        drivers_port_finder::GetPort srv;
        srv.request.component = "usb2ax";

        if (client.call(srv))
        {
            port = srv.response.port;
        }
        else
        {
            ROS_ERROR("Failed to fetch the AX-12 port from drivers_port_finder");
        }
    }

    if(port.length() == 0)
    {
        ROS_ERROR("The AX-12 port is not set, falling back to default: %s", DEFAULT_PORT.c_str());
        port = DEFAULT_PORT;
    }

    return port;
}

const Ax12Table::Register* Ax12Server::service_param_to_register(uint8_t param)
{
    switch(param)
    {
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
    }

    return nullptr;
}

bool Ax12Server::execute_set_service_cb(drivers_ax12::SetAx12Param::Request &req,
                                        drivers_ax12::SetAx12Param::Response &res)
{

    if(!driver_.motor_id_exists(req.motor_id))
    {
        ROS_ERROR("AX-12 set_param service server received a request for motor ID %d, but no such motor was detected", req.motor_id);
        res.success = 0;
        return true;
    }

    if(!driver_.motor_id_connected(req.motor_id))
    {
        ROS_ERROR("AX-12 set_param service server received a request for motor ID %d, but the motor was disconnected", req.motor_id);
        res.success = 0;
        return true;
    }


    const Register* reg = service_param_to_register(req.param);

    if(reg == nullptr)
    {
        ROS_ERROR("AX-12 set_param service server received a invalid param request");
        res.success = 0;
        return true;
    }

    ROS_INFO("Successfully changed parameter %d of motor with ID %d to %d", req.param, req.motor_id, req.value);

    res.success = (uint8_t)driver_.write_register(req.motor_id, *reg, req.value);
    return true;
}

Ax12Server::Ax12Server(std::string action_name, std::string service_name) :
        as_(nh_, action_name, boost::bind(&Ax12Server::execute_goal_cb, this, _1), false),
        set_param_service(nh_.advertiseService(service_name, &Ax12Server::execute_set_service_cb, this)),
        driver_(),
        joint_goals_(),
        feedback_(),
        result_()
{
    std::string port;

    as_.start();

    port = fetch_port(PORT_FINDER_SERVICE);

    init_driver(port);

    ROS_INFO("AX-12 action server initialized for port %s, waiting for goals", port.c_str());

    timer_ = nh_.createTimer(ros::Duration(1.0/MAIN_FREQUENCY), &Ax12Server::main_loop, this);

}

Ax12Server::~Ax12Server()
{
    driver_.toggle_torque(false);
    ros::shutdown();
}