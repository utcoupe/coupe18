cmake_minimum_required(VERSION 2.8.3)
project(movement_navigation_navigator)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    rospy
    std_msgs
    geometry_msgs
    pathfinding
    asserv
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate services in the 'srv' folder
add_service_files(
    FILES
    Goto.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
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

