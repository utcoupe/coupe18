cmake_minimum_required(VERSION 2.8.3)
project(recognition_objects_classifier)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    message_generation
    geometry_msgs
    processing_lidar_objects
    processing_belt_interpreter
    std_msgs
    tf2
    tf2_ros
    rospy
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
    FILES
    CircleObstacleStamped.msg
    ClassifiedObjects.msg
    SegmentObstacleStamped.msg
)

generate_messages(
    DEPENDENCIES
    processing_lidar_objects
    processing_belt_interpreter
    geometry_msgs
    std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
    CATKIN_DEPENDS message_runtime roscpp
    INCLUDE_DIRS include
    LIBRARIES recognition_objects_classifier
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(objects_classifier_node src/objects_classifier_node.cpp src/objects_listener.cpp src/shapes.cpp)

add_dependencies(objects_classifier_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(
    objects_classifier_node
    ${catkin_LIBRARIES}
)