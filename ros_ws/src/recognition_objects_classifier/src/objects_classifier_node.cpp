#include "ros/ros.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "objects_classifier");

    ros::NodeHandle node_handle();
    ros::spin();
    return 0;
}
