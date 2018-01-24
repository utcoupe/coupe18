#include "drivers_ax12/ax12_server.h"


void Ax12Server::init_workbench(const std::string& port)
{
    /*
     * - initializes the DynamixelWorkbench
     * - scans the motors
     * - filter the motors to keep only those configured in the ROS params
     * - configure those (sets the init, min and max position)
     */
    uint8_t nbr_of_ids;
    uint8_t ids[SCAN_RANGE];
    std::string param_key;

    if(!dxl_wb_.begin(port.c_str(), BAUD_RATE)) {
        ROS_FATAL("Unable to initialize the AX-12 dynamixel workbench (port: %s, baudrate: %d)", port.c_str(), BAUD_RATE);
        ros::shutdown();
        return;
    }

    if(!dxl_wb_.scan(ids, &nbr_of_ids, SCAN_RANGE))
    {
        ROS_ERROR("Unable to scan the AX-12 motors on %s", port.c_str());
    }

    ROS_INFO("Found %d AX-12 motors connected with a scan range of %d", nbr_of_ids, SCAN_RANGE);


    for(uint8_t i = 0; i < nbr_of_ids; i++)
    {
        param_key = ros::names::append("motors", std::to_string(ids[i]));
        param_key = ros::names::append(param_key, "init");

        if(!ros::param::has(param_key))
        {
            ROS_WARN("No definition for AX-12 with ID %d, it will be ignored", ids[i]);
        }
        else
        {
            ROS_INFO("Definition for AX-12 with ID %d found, configuring...", ids[i]);
            init_motor(ids[i]);
            dxl_id_[dxl_cnt_++] = ids[i];

        }
    }
}

bool Ax12Server::init_motor(uint8_t motor_id)
{
    /*
     * Sets the Present_Position, CW_Angle_Limit and CCW_Angle_Limit of a motor
     * according to ros params. If params are not found, it will fall back to default
     * constants
     */

    int param_value = 0;
    bool write_success = true;

    std::string base_key = ros::names::append("motors", std::to_string(motor_id));

    write_success &= dxl_wb_.jointMode(motor_id, DEFAULT_VEL, 0);




    // init pos
    if(ros::param::get(ros::names::append(base_key, "init"), param_value))
    {
        write_success &= dxl_wb_.itemWrite(motor_id, "Present_Position", param_value);
    }
    else
    {
        ROS_ERROR("AX-12 with ID %d need a 'init' value in the definition file", motor_id);
    }

    // min pos
    if(ros::param::get(ros::names::append(base_key, "min"), param_value))
    {
        write_success &= dxl_wb_.itemWrite(motor_id, "CW_Angle_Limit", param_value);
    }
    else
    {
        ROS_WARN("AX-12 with ID %d doesn't have a 'min' value in the definition file, falling back to default: %d", motor_id, DEFAULT_MIN);
        write_success &= dxl_wb_.itemWrite(motor_id, "CW_Angle_Limit", DEFAULT_MIN);
    }

    // max pos
    if(ros::param::get(ros::names::append(base_key, "max"), param_value))
    {
        write_success &= dxl_wb_.itemWrite(motor_id, "CCW_Angle_Limit", param_value);
    }
    else
    {
        ROS_ERROR("AX-12 with ID %d doesn't have a 'max' value in the definition file, falling back to default: %d", motor_id, DEFAULT_MAX);
        write_success &= dxl_wb_.itemWrite(motor_id, "CCW_Angle_Limit", DEFAULT_MAX);
    }

    if(!write_success)
    {
        ROS_ERROR("Errors occured when configuring motor with id %d", motor_id);
    }

    return write_success;

}

bool Ax12Server::motor_id_exists(uint8_t motor_id)
{
    /*
     * Returns true if the motor was successfully detected and configured
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

    int32_t min = dxl_wb_.itemRead(motor_id, "CW_Angle_Limit");
    int32_t max = dxl_wb_.itemRead(motor_id, "CCW_Angle_Limit");

    if(min < max)
    {
        return (position >= min && position <= max);
    }
    else
    {
        return (position >= max && position <= min);
    }

}

void Ax12Server::execute_goal_cb(GoalHandle goal_handle)
{
    /*
     * Handles the goal requests of the action server.
     * Checks if the goal is valid, then tells the motor
     * to move and adds the goal to the list of current goals
     */

    bool success = true;

    if(!goal_handle.isValid())
    {
        ROS_ERROR("AX-12 action server received an invalid goal !");
        goal_handle.setRejected();
        return;
    }

    uint8_t motor_id = goal_handle.getGoal()->motor_id;

    if(!motor_id_exists(motor_id))
    {
        ROS_ERROR("AX-12 action server received a goal for motor ID %d, but no such motor was detected", motor_id);
        goal_handle.setRejected();
        return;
    }

    uint16_t position = goal_handle.getGoal()->position;
    if(!position_is_valid(motor_id, position))
    {
        ROS_ERROR("AX-12 action server received a goal for motor ID %d, but with an invalid position: %d", motor_id, position);
        goal_handle.setRejected();
        return;
    }

    uint16_t speed = goal_handle.getGoal()->speed;
    if(speed < 0 || speed > 1023)
    {
        ROS_ERROR("AX-12 action server received a goal for motor ID %d, but with an invalid speed: %d", motor_id, speed);
        goal_handle.setRejected();
        return;
    }

    goal_handle.setAccepted();

    for(auto it = current_goals.begin(); it != current_goals.end();)
    {
        if(it->getGoal()->motor_id == motor_id)
        {
            it->setCanceled();
            ROS_INFO("AX-12 action server received a goal for motor ID %d while another goal was running for that motor. The old goal was canceled.", motor_id);
            it = current_goals.erase(it);
        }
        else
        {
            it++;
        }
    }

    success &= dxl_wb_.goalSpeed(motor_id, speed);
    success &= dxl_wb_.goalPosition(motor_id, position);

    if(!success) {
        goal_handle.setAborted();
    }
    else
    {
        current_goals.push_back(goal_handle);
    }

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

    for(auto it = current_goals.begin(); it != current_goals.end();)
    {
        motor_id = it->getGoal()->motor_id;
        curr_position = dxl_wb_.itemRead(motor_id, "Present_Position");

        if(dxl_wb_.itemRead(motor_id, "Moving"))
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
            it = current_goals.erase(it);
            ROS_INFO("AX-12 position goal succeeded for motor ID %d", motor_id);

        }
        else if(ros::Time::now().toSec() - it->getGoalID().stamp.toSec() > MAX_STOP_TIME)
        {
            result_.success = false;
            it->setAborted();
            it = current_goals.erase(it);
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

std::string Ax12Server::fetch_def_file_path(const std::string& service_name)
{
    /*
     * Asks the definitions for the path of the definition file
     */

    std::string path;

    if(!ros::service::waitForService(service_name, 10000))
    {
        ROS_ERROR("Failed to contact memory_definitions (service not up)");
    }
    else
    {
        ros::ServiceClient client = nh_.serviceClient<memory_definitions::GetDefinition>(service_name);
        memory_definitions::GetDefinition srv;
        srv.request.request = DEFINITION_PATH;

        if (client.call(srv) && srv.response.success)
        {
            return srv.response.path;
        }
        else
        {
            ROS_ERROR("Failed to fetch the AX-12 definition file from memory_definitions");
        }
    }

    if(path.length() == 0)
    {
        ROS_ERROR("The AX-12 definition file path is not set");
    }

    return path;

}

bool Ax12Server::set_ros_params(const std::string& def_file_path)
{
    /*
     * Calls rosparam to set params from the definition file
     */

    std::string ns = ros::names::append(nh_.getNamespace(), "motors");

    int needed = snprintf(nullptr, 0, "rosparam load %s %s",  def_file_path.c_str(), ns.c_str()) + 1;
    if(needed < 0)
    {
        ROS_ERROR("Unable to load AX-12 parameters from definition file: can't build rosparam command");
    }

    char command[needed];

    snprintf(command, (size_t)needed, "rosparam load %s %s", def_file_path.c_str(), ns.c_str());

    int code = system(command);

    if(code)
    {
        ROS_ERROR("Unable to load AX-12 parameters from definition file: rosparam failed");
    }

    return (code == 0);
}

Ax12Server::Ax12Server(std::string name) :
        as_(nh_, name, boost::bind(&Ax12Server::execute_goal_cb, this, _1), false)
{
    std::string def_file_path, port;

    as_.start();

    def_file_path = fetch_def_file_path(DEFINITIONS_SERVICE);
    set_ros_params(def_file_path);

    port = fetch_port(PORT_FINDER_SERVICE);
    init_workbench(port);

    ROS_INFO("AX-12 action server initialized for port %s, waiting for goals", port.c_str());

    ros::Timer timer = nh_.createTimer(ros::Duration(1.0/MAIN_FREQUENCY), boost::bind(&Ax12Server::main_loop, this, _1));

}

Ax12Server::~Ax12Server()
{

    //for (int index = 0; index < dxl_cnt_; index++)
    //    dxl_wb_.itemWrite(dxl_id_[index], "Torque_Enable", 0);

    ros::shutdown();
}