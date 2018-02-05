cmake_minimum_required(VERSION 2.8.3)
project(navigation_navigator)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    rospy
    std_msgs
    geometry_msgs
    actionlib
    actionlib_msgs
    navigation_pathfinder
    drivers_ard_asserv
    memory_map
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
    FILES
    Status.msg
)

## Generate services in the 'srv' folder
add_service_files(
    FILES
    Goto.srv
)

## Generate actions in the 'action' folder
add_action_files(
    FILES
    DoGoto.action
    DoGotoWaypoint.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
    actionlib_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################

catkin_package(
#    CATKIN_DEPENDS rospy std_msgs message_runtime
)

###########
## Build ##
###########

#############
## Install ##
#############

#############
## Testing ##
#############

