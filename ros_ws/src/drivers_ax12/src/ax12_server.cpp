#include "drivers_ax12/ax12_server.h"


void Ax12Server::init_workbench(const std::string& port)
{
    if(!dxl_wb_.begin(port.c_str(), BAUD_RATE)) {
        ROS_FATAL("Error initializing dynamixel workbench (port: %s, baudrate: %d), shutting down...", port.c_str(), BAUD_RATE);
        ros::shutdown();
    }


    dxl_wb_.scan(dxl_id_, &dxl_cnt_, SCAN_RANGE);
    ROS_INFO("Found %d ax12 connected ! (scan range: %d)", dxl_cnt_, SCAN_RANGE);


    for (uint8_t index = 0; index < dxl_cnt_; index++)
        dxl_wb_.jointMode(dxl_id_[index], DEFAULT_VEL, 0); // no acceleration for ax12a
}

void Ax12Server::intersect_id_lists() //drops ids in scanned ids that are not in ros params
{

    std::string key;

    for(uint8_t i = dxl_cnt_; i >= 0; i--)
    {

        key = ros::names::append("motors", std::to_string(i));
        key = ros::names::append(key, "init");

        if(!ros::param::has(key))
        {
            //todo: delete from list
            ROS_WARN("Detected ax12 with id %d, no such id in definition file, ax12 will be ignored", i);
        }
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

    goal_handle.setAccepted();

    current_goals.insert(std::make_pair(goal_handle.getGoalID().id.c_str(), goal_handle));


    ROS_INFO("%s", goal_handle.getGoalID().id.c_str());
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

void Ax12Server::set_ros_params(const std::string& def_file_path)
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
    init_workbench(port);



    ROS_INFO("drivers_ax12 action server initialized (port %s), waiting for goals...", port.c_str());
}

Ax12Server::~Ax12Server()
{
    for (int index = 0; index < dxl_cnt_; index++)
        dxl_wb_.itemWrite(dxl_id_[index], "Torque_Enable", 0);

    ros::shutdown();
}