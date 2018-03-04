cmake_minimum_required(VERSION 2.8.3)
project(navigation_pathfinder)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    genmsg
    geometry_msgs
    dynamic_reconfigure
    processing_belt_interpreter
    processing_lidar_objects
    ai_game_status
    memory_map
)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules" ${CMAKE_MODULE_PATH}) # Please note the use of CMAKE_CURRENT_SOURCE_DIR
find_package(SFML 2 REQUIRED COMPONENTS
    graphics
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_service_files(
    FILES
    FindPath.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

generate_dynamic_reconfigure_options(
    def/PathfinderNode.cfg
)

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
    ${catkin_INCLUDE_DIRS}
)


file(
    GLOB_RECURSE
    source_files_pathfinder
    src/pathfinder/*
)
add_executable(pathfinder_node ${source_files_pathfinder} src/pathfinder_node.cpp)

target_link_libraries(pathfinder_node
    ${catkin_LIBRARIES}
    ${SFML_LIBRARIES} ${SFML_DEPENDENCIES}
)

add_dependencies(pathfinder_node
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${PROJECT_NAME}_gencfg
    ${catkin_EXPORTED_TARGETS}
)

#############
## Install ##
#############


#############
## Testing ##
#############

