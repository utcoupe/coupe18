cmake_minimum_required(VERSION 2.8.3)
project(ai_game_status)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
    rospy
    roscpp
    std_msgs
    ai_scheduler
    message_generation
)

catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
    FILES
    GameStatus.msg
    NodesStatus.msg
    ArmRequest.msg
)

add_service_files(
    FILES
    SetStatus.srv
    NodeReady.srv
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
    INCLUDE_DIRS include
    CATKIN_DEPENDS message_runtime
    LIBRARIES ${PROJECT_NAME}
)

###########
## Build ##
###########

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})

add_library(
    ${PROJECT_NAME}
    src/init_service.cpp
)
target_link_libraries(
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
)

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp)

#add_dependencies(
#    ${${PROJECT_NAME}_EXPORTED_TARGETS}
#    ${${PROJECT_NAME}_generate_messages_cpp}
#    ${catkin_EXPORTED_TARGETS}
#)

#############
## Install ##
#############


#############
## Testing ##
#############
