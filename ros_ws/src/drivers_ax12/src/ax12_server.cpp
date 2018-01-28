#include "ax12_server.h"
#include "dynamixel.h"

void Ax12Server::init_workbench(const std::string& port)
{
    /*
     * - initializes the DynamixelWorkbench
     * - scans the motors
     */

    std::string param_key;
    int8_t port_index;
    size_t port_num_index = port.find_last_of("0123456789");
    port_index = port[port_num_index] - '0';

    if(port_num_index == std::string::npos || port_index < 0 || port_index > 9) {
        ROS_ERROR("Unable to get the port index from the port %s", port.c_str());
    }

    if(!dxl_initialize(port_index, BAUD_RATE_INDEX)) {
        ROS_FATAL("Unable to initialize the AX-12 library (port index: %d, baudrate index: %d)", port_index, BAUD_RATE_INDEX);
        ros::shutdown();
        return;
    }

    for(uint8_t i = 1; i <= SCAN_RANGE; i++) {
        ROS_DEBUG("Pinging AX-12 with id %d", i)
        for(uint8_t j = 0; j < 100; j++) {
            dxl_ping(i);
            usleep(500);
            if (dxl_get_result() == COMM_RXSUCCESS) {
                ROS_DEBUG("AX-12 detected with id %d", i);
                dxl_id_[dxl_cnt_++] = i;
                break;
            }
        }

    }

    ROS_INFO("Found %d AX-12 motors connected with a scan range of %d", dxl_cnt_, SCAN_RANGE);


    for (int index = 0; index < dxl_cnt_; index++)
        dxl_write_byte(dxl_id_[index], TORQUE_ENABLE_ADDR, 1);


}

bool Ax12Server::motor_id_exists(uint8_t motor_id)
{
    /*
     * Returns true if the motor was successfully detected
     */

    for(uint8_t i = 0; i < dxl_cnt_; i++)
    {
        if(dxl_id_[i] == motor_id)
            return true;
    }

    return false;
}

bool Ax12Server::position_is_valid(uint8_t motor_id, uint16_t position)
{
    /*
     * Returns true if the position is within the range [min; max]
     */

    return position >= 0 && position <= 1023;

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

    if(!motor_id_exists(motor_id))
    {
        ROS_ERROR("AX-12 action server received a goal for motor ID %d, but no such motor was detected", motor_id);
        goal_handle.setRejected();
        return;
    }

    if(goal->mode == goal->JOINT)
    {
        handle_joint_goal(goal_handle);
    }
    else
    {
        handle_wheel_goal(goal_handle);
    }
}

bool Ax12Server::handle_joint_goal(GoalHandle goal_handle)
{
    auto goal = goal_handle.getGoal();
    uint8_t motor_id = goal->motor_id;
    uint16_t position = goal->position;
    bool success;

    if(!position_is_valid(motor_id, position))
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

    for(auto it = joint_goals.begin(); it != joint_goals.end();)
    {
        if(it->getGoal()->motor_id == motor_id)
        {
            it->setCanceled();
            ROS_INFO("AX-12 action server received a joint goal for motor ID %d while another joint goal was running for that motor. The old goal was canceled.", motor_id);
            it = joint_goals.erase(it);
        }
        else
        {
            it++;
        }
    }

    //set joint mode
    dxl_write_byte(motor_id, TORQUE_ENABLE_ADDR, 0);
    dxl_write_word(motor_id, CW_ANGLE_LIMIT_ADDR, 1); //todo: set to 0
    dxl_write_word(motor_id, CCW_ANGLE_LIMIT_ADDR, 1023);
    dxl_write_byte(motor_id, TORQUE_ENABLE_ADDR, 1);
    dxl_write_word(motor_id, MOVING_SPEED_ADDR, speed);
    dxl_write_word(motor_id, GOAL_POSITION_ADDR, position);

    success = dxl_get_result() == COMM_TXSUCCESS || dxl_get_result() == COMM_RXSUCCESS;

    if(!success) {
        goal_handle.setAborted();
    }
    else
    {
        joint_goals.push_back(goal_handle);
    }

    return success;
}

bool Ax12Server::handle_wheel_goal(GoalHandle goal_handle)
{
    auto goal = goal_handle.getGoal();
    uint8_t motor_id = goal->motor_id;
    uint16_t speed = goal->speed;
    bool success;

    if(speed < 0 || speed > 2047)
    {
        ROS_ERROR("AX-12 action server received a wheel goal for motor ID %d, but with an invalid speed: %d", motor_id, speed);
        goal_handle.setRejected();
        return false;
    }

    goal_handle.setAccepted();

    for(auto it = joint_goals.begin(); it != joint_goals.end();)
    {
        if(it->getGoal()->motor_id == motor_id)
        {
            it->setCanceled();
            ROS_INFO("AX-12 action server received a wheel goal for motor ID %d while another joint goal was running for that motor. The old goal was canceled.", motor_id);
            it = joint_goals.erase(it);
        }
        else
        {
            it++;
        }
    }

    // wheel mode
    dxl_write_byte(motor_id, TORQUE_ENABLE_ADDR, 0);
    dxl_write_word(motor_id, CW_ANGLE_LIMIT_ADDR, 0);
    dxl_write_word(motor_id, CCW_ANGLE_LIMIT_ADDR, 0);
    dxl_write_byte(motor_id, TORQUE_ENABLE_ADDR, 1);
    dxl_write_word(motor_id, MOVING_SPEED_ADDR, speed);

    success = dxl_get_result() == COMM_TXSUCCESS || dxl_get_result() == COMM_RXSUCCESS;

    if(!success) {
        goal_handle.setAborted();
    } else
    {
        goal_handle.setSucceeded();
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
    int32_t curr_position;
    int32_t goal_position;
    ros::Rate r(MAIN_FREQUENCY);

    for(auto it = joint_goals.begin(); it != joint_goals.end();)
    {
        motor_id = it->getGoal()->motor_id;
        curr_position = dxl_read_word(motor_id, PRESENT_POSITION_ADDR);

        if(dxl_read_byte(motor_id, MOVING_ADDR))
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
            it->setSucceeded();
            it = joint_goals.erase(it);
            ROS_INFO("AX-12 position goal succeeded for motor ID %d", motor_id);

        }
        else if(ros::Time::now().toSec() - it->getGoalID().stamp.toSec() > MAX_STOP_TIME)
        {
            result_.success = false;
            it->setAborted();
            it = joint_goals.erase(it);
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
        as_(nh_, name, boost::bind(&Ax12Server::execute_goal_cb, this, _1), false)
{
    std::string port;

    as_.start();

    port = fetch_port(PORT_FINDER_SERVICE);
    init_workbench(port);

    ROS_INFO("AX-12 action server initialized for port %s, waiting for goals", port.c_str());

    ros::Timer timer = nh_.createTimer(ros::Duration(1.0/MAIN_FREQUENCY), boost::bind(&Ax12Server::main_loop, this, _1));

}

Ax12Server::~Ax12Server()
{
    for (int index = 0; index < dxl_cnt_; index++)
        dxl_write_byte(dxl_id_[index], TORQUE_ENABLE_ADDR, 0);

    ros::shutdown();
}