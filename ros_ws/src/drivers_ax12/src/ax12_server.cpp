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


    if(!ros::service::waitForService(service_name, 10000))
    {
        ROS_ERROR("Failed to contact drivers_port_finder (service not up)");
        return "";
    }

    ros::ServiceClient client = nh_.serviceClient<drivers_port_finder::GetPort>(service_name);
    drivers_port_finder::GetPort srv;
    srv.request.component = "usb2ax";

    if (client.call(srv))
    {
        return srv.response.port;
    }
    else
    {
        ROS_ERROR("Failed to fetch the port from drivers_port_finder");
        return "";
    }
}
