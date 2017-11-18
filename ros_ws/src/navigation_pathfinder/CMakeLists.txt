cmake_minimum_required(VERSION 2.8.3)
project(navigation_pathfinder)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    genmsg
    geometry_msgs
)

find_package(Boost REQUIRED)

SET(TCLAP_INCLUDE_DIR $ENV{UTCOUPE_WORKSPACE}/libs/tclap-1.2.1/include)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate services in the 'srv' folder
add_service_files(
    FILES
    DoOrder.srv
    FindPath.srv
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
    CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

include_directories(
    include
    ${TCLAP_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
)

file(
    GLOB_RECURSE
    source_files_pathfinder
    src/pathfinder/*
)

add_executable(pathfinder_node ${source_files_pathfinder} src/pathfinder.cpp)

target_link_libraries(pathfinder_node ${catkin_LIBRARIES})

add_dependencies(pathfinder_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

#############
## Testing ##
#############
