#include "drivers_ax12/ax12_server.h"


void Ax12Server::init_workbench(const std::string& port)
{
    if(!dxl_wb_.begin(port.c_str(), BAUD_RATE)) {
        ROS_FATAL("Error initializing dynamixel workbench (port: %s, baudrate: %d), shutting down...", port.c_str(), BAUD_RATE);
        ros::shutdown();
        return;
    }

    uint8_t nbr_of_ids;
    uint8_t ids[SCAN_RANGE];

    if(!dxl_wb_.scan(ids, &nbr_of_ids, SCAN_RANGE))
    {
        ROS_ERROR("Scan of ax12 motors failed !");
    }

    ROS_INFO("Found %d ax12 connected ! (scan range: %d)", nbr_of_ids, SCAN_RANGE);

    std::string param_key;
    for(uint8_t i = 0; i < nbr_of_ids; i++)
    {

        param_key = ros::names::append("motors", std::to_string(ids[i]));
        param_key = ros::names::append(param_key, "init");

        if(!ros::param::has(param_key))
        {
            ROS_WARN("Detected ax12 with id %d, no such id in definition file, ax12 will be ignored", i);
        }
        else
        {
            init_motor(ids[i]);
            dxl_id_[dxl_cnt_++] = ids[i];
            ROS_INFO("ax12 with id %d detected and configured !", ids[i]);
        }
    }
}

bool Ax12Server::init_motor(uint8_t motor_id)
{

    bool write_success = true;
    std::string base_key = ros::names::append("motors", std::to_string(motor_id));

    write_success &= dxl_wb_.jointMode(motor_id, DEFAULT_VEL, 0);

    int param_value = 0;


    // init pos
    if(ros::param::get(ros::names::append(base_key, "init"), param_value))
    {
        write_success &= dxl_wb_.itemWrite(motor_id, "Present_Position", param_value);
    }
    else
    {
        ROS_ERROR("Motor with id %d need a 'init' value in the definition file !", motor_id);
    }

    // min pos
    if(ros::param::get(ros::names::append(base_key, "min"), param_value))
    {
        write_success &= dxl_wb_.itemWrite(motor_id, "CW_Angle_Limit", param_value);
    }
    else
    {
        ROS_WARN("Motor with id %d doesn't have a 'min' value in the definition file, falling back to default", motor_id);
        write_success &= dxl_wb_.itemWrite(motor_id, "CW_Angle_Limit", DEFAULT_MIN);
    }

    // max pos
    if(ros::param::get(ros::names::append(base_key, "max"), param_value))
    {
        write_success &= dxl_wb_.itemWrite(motor_id, "CCW_Angle_Limit", param_value);
    }
    else
    {
        ROS_ERROR("Motor with id %d doesn't have a 'max' value in the definition file, falling back to default", motor_id);
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
    for(uint8_t i = 0; i < dxl_cnt_; i++)
    {
        if(dxl_id_[i] == motor_id)
            return true;
    }

    return false;
}

bool Ax12Server::position_is_valid(uint8_t motor_id, uint16_t position)
{
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
    if(!goal_handle.isValid())
    {
        ROS_ERROR("Ax12 received invalid goal !");
        goal_handle.setRejected();
        return;
    }

    uint8_t motor_id = goal_handle.getGoal()->motor_id;

    if(!motor_id_exists(motor_id))
    {
        ROS_ERROR("Received goal for motor id %d, but no such motor was detected", motor_id);
        goal_handle.setRejected();
        return;
    }

    uint16_t position = goal_handle.getGoal()->position;
    if(!position_is_valid(motor_id, position))
    {
        ROS_ERROR("Received goal for motor id %d, but with an invalid position: %d", motor_id, position);
        goal_handle.setRejected();
        return;
    }

    uint16_t speed = goal_handle.getGoal()->speed;
    if(speed < 0 || speed > 1023)
    {
        ROS_ERROR("Received goal for motor id %d, but with an invalid speed: %d", motor_id, speed);
        goal_handle.setRejected();
        return;
    }

    goal_handle.setAccepted();

    bool success = dxl_wb_.goalSpeed(motor_id, speed);
    success &= dxl_wb_.goalPosition(motor_id, position);

    if(!success) {
        goal_handle.setAborted();
    }
    else
    {
        current_goals.insert(std::make_pair(goal_handle.getGoalID().id.c_str(), goal_handle));
    }

}



std::string Ax12Server::fetch_port(const std::string& service_name)
{
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
            ROS_ERROR("Failed to fetch the port from drivers_port_finder");
        }
    }

    if(port.length() == 0)
    {
        ROS_ERROR("The port is not set, falling back to default %s", DEFAULT_PORT.c_str());
        port = DEFAULT_PORT;
    }

    return port;
}

std::string Ax12Server::fetch_def_file_path(const std::string& service_name)
{

    std::string path;

    if(!ros::service::waitForService(service_name, 10000))
    {
        ROS_ERROR("Failed to contact memory_definitions (service not up)");
    }
    else
    {
        ros::ServiceClient client = nh_.serviceClient<memory_definitions::GetDefinition>(service_name);
        memory_definitions::GetDefinition srv;
        srv.request.request = "drivers/ax12.yaml";

        if (client.call(srv) && srv.response.success)
        {
            return srv.response.path;
        }
        else
        {
            ROS_ERROR("Failed to fetch the definition file from memory_definitions");
        }
    }

    if(path.length() == 0)
    {
        ROS_ERROR("The definition file path is not set !");
    }

    return path;

}

bool Ax12Server::set_ros_params(const std::string& def_file_path)
{
    std::string ns = ros::names::append(nh_.getNamespace(), "motors");

    int needed = snprintf(nullptr, 0, "rosparam load %s %s",  def_file_path.c_str(), ns.c_str()) + 1;
    if(needed < 0)
    {
        ROS_ERROR("Cannot load parameters from definition file !");
    }

    char command[needed];

    snprintf(command, (size_t)needed, "rosparam load %s %s", def_file_path.c_str(), ns.c_str());

    int code = system(command);

    if(code)
    {
        ROS_ERROR("Cannot load parameters from definition file !");
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


    // port = fetch_port(PORT_FINDER_SERVICE);


    port = "/dev/ttyACM0";
    //init_workbench(port);



    ROS_INFO("drivers_ax12 action server initialized (port %s), waiting for goals...", port.c_str());
}

Ax12Server::~Ax12Server()
{

    for (int index = 0; index < dxl_cnt_; index++)
        dxl_wb_.itemWrite(dxl_id_[index], "Torque_Enable", 0);

    ros::shutdown();
}