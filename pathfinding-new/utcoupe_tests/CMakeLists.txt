cmake_minimum_required(VERSION 2.8.3)
project(utcoupe_tests)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs genmsg)

find_package(Boost REQUIRED)

SET(TCLAP_INCLUDE_DIR libs/tclap-1.2.1/include)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate services in the 'srv' folder
add_service_files(
  FILES
  FindPath.srv
)

## Generate added messages and services with any dependencies listed here
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
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    ${TCLAP_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
file(
    GLOB_RECURSE
    source_files_pathfinding
    src/pathfinding/*
)
add_executable(pathfinding ${source_files_pathfinding} src/pathfinding.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(pathfinding ${catkin_LIBRARIES})

#############
## Install ##
#############



#############
## Testing ##
#############


