cmake_minimum_required(VERSION 2.8.3)
project(drivers_ax12)

find_package(catkin REQUIRED COMPONENTS
    rospy
    actionlib
    actionlib_msgs
    message_generation)


## Generate actions
add_action_files(DIRECTORY action FILES AngleCommand.action)

# Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    actionlib_msgs
)

catkin_package(
    CATKIN_DEPENDS
    message_runtime
    dynamixel_controllers
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)
