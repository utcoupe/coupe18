cmake_minimum_required(VERSION 2.8.3)
project(recognition_objects_classifier)

find_package(catkin REQUIRED COMPONENTS
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
    CATKIN_DEPENDS message_runtime
)
