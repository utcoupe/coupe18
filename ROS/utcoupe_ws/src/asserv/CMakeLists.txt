cmake_minimum_required(VERSION 2.8.3)
project(asserv)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        rospy
        std_msgs
        geometry_msgs
        message_generation
)

catkin_python_setup()

## Generate messages in the 'msg' folder
add_message_files(
        FILES
        RobotSpeed.msg
)

## Generate services in the 'srv' folder
add_service_files(
        FILES
        Goto.srv
        AsservParameters.srv
)

# Generate added messages and services with any dependencies listed here
generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
)

catkin_package(
        CATKIN_DEPENDS message_runtime
)

include_directories(
        ${catkin_INCLUDE_DIRS}
)