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
    bool success = true;

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

    success &= driver_.joint_mode(motor_id);
    success &= driver_.write_register(motor_id, MOVING_SPEED, speed);
    success &= driver_.write_register(motor_id, GOAL_POSITION, position);

    if(!success)
        goal_handle.setAborted();
    else
        joint_goals_.push_back(goal_handle);

    return success;
}

bool Ax12Server::handle_wheel_goal(GoalHandle goal_handle)
{
    auto goal = goal_handle.getGoal();
    uint8_t motor_id = goal->motor_id;
    uint16_t speed = goal->speed;
    bool success = true;

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

    success &= driver_.wheel_mode(motor_id);
    success &= driver_.write_register(motor_id, MOVING_SPEED, speed);

    if(!success) {
        goal_handle.setAborted();
    }
    else
    {
        result_.success = true;
        goal_handle.setSucceeded(result_);
    }

    return success;
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

    for(auto it = joint_goals_.begin(); it != joint_goals_.end();)
    {
        motor_id = it->getGoal()->motor_id;
        driver_.read_register(motor_id, PRESENT_POSITION, curr_position);
        driver_.read_register(motor_id, MOVING, moving);
        if(moving)
        {
            feedback_.position = curr_position;
            it->publishFeedback(feedback_);
            it++;
            continue;
        }

        goal_position = it->getGoal()->position;

        if(curr_position == goal_position)
        {
            result_.success = true;
            it->setSucceeded(result_);
            it = joint_goals_.erase(it);
            ROS_INFO("AX-12 position goal succeeded for motor ID %d", motor_id);

        }
        else if(ros::Time::now().toSec() - it->getGoalID().stamp.toSec() > MAX_STOP_TIME)
        {
            result_.success = false;
            it->setAborted(result_);
            it = joint_goals_.erase(it);
            ROS_ERROR("AX-12 position goal aborted for motor ID %d", motor_id);
        }
        else
        {
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

    if(!ros::service::waitForService(service_name, 10000))
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

Ax12Server::Ax12Server(std::string name) :
        as_(nh_, name, boost::bind(&Ax12Server::execute_goal_cb, this, _1), false),
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

    ros::Timer timer = nh_.createTimer(ros::Duration(1.0/MAIN_FREQUENCY), boost::bind(&Ax12Server::main_loop, this, _1));

}

Ax12Server::~Ax12Server()
{
    driver_.toggle_torque(false);
    ros::shutdown();
}