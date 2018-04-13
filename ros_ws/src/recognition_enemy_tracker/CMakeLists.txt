cmake_minimum_required(VERSION 2.8.3)
project(recognition_enemy_tracker)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    rospy
    std_msgs
    geometry_msgs
    ai_game_status
    recognition_objects_classifier
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
    FILES
    EnemyStamped.msg
    Enemies.msg
)

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
