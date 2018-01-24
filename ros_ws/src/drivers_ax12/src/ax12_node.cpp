#include "ros/ros.h"
#include "drivers_ax12/ax12_server.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ax12");
    Ax12Server server("angle_command");
    ros::spin();
    return 0;
}

