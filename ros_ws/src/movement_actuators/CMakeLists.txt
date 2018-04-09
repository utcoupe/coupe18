cmake_minimum_required(VERSION 2.8.3)
project(movement_actuators)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    actionlib_msgs
)

## Generate actions in the 'action' folder
add_action_files(DIRECTORY action FILES
        dispatch.action
        arm.action
        barrel.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    actionlib_msgs
    std_msgs
)

catkin_package(
    CATKIN_DEPENDS actionlib_msgs rospy
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)
