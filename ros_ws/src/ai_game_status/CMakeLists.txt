cmake_minimum_required(VERSION 2.8.3)
project(ai_game_status)

find_package(catkin REQUIRED COMPONENTS
    rospy
    std_msgs
    ai_scheduler
    message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
    FILES
    GameStatus.msg
    NodesStatus.msg
    GameTime.msg
)

add_service_files(
    FILES
    SetStatus.srv
    ArmRequest.srv
    NodeReady.srv
    SetTimer.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################

catkin_package(
    CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

include_directories(
    ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############


#############
## Testing ##
#############
