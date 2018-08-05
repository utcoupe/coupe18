cmake_minimum_required(VERSION 2.8.3)
project(navigation_collisions)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  std_msgs
  genmsg
  geometry_msgs
  tf2
  tf2_ros
  ai_game_status
  memory_map
  recognition_objects_classifier
)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules" ${CMAKE_MODULE_PATH}) # Please note the use of CMAKE_CURRENT_SOURCE_DIR

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  FILES
  PredictedCollision.msg
)

add_service_files(
  FILES
  ActivateCollisions.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS roscpp message_runtime
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(collisions_node src/collisions_node.cpp)

target_link_libraries(
  collisions_node
  ${catkin_LIBRARIES}
)

add_dependencies(
  collisions_node 
  navigation_collisions_generate_messages_cpp 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)
