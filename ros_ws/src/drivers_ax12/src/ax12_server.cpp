#include "drivers_ax12/ax12_server.h"

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

std::string Ax12Server::fetch_port()
{
    std::string service_name = "/drivers/port_finder/get_port";

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
        ROS_FATAL("The port is not set, shutting down...");
        ros::shutdown();
    }

    return port;
}

std::string Ax12Server::fetch_def_file_path()
{
    std::string service_name = "/memory/definitions/get";

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
        ROS_FATAL("The definition file path is not set, shutting down...");
        ros::shutdown();
    }

    return path;

}

void Ax12Server::set_ros_params()
{
    std::string ns = ros::names::append(nh_.getNamespace().c_str(), "motors");

    size_t needed = snprintf(NULL, 0, "rosparam load %s %s",  def_file_path_.c_str(), ns.c_str()) + 1;
    char command[needed];

    snprintf(command, needed, "rosparam load %s %s", def_file_path_.c_str(), ns.c_str());
    ROS_ERROR(command);

    int code = system(command);

    if(code)
    {
        ROS_ERROR("Couln't set the ROS params from the definition file");
    }
}
