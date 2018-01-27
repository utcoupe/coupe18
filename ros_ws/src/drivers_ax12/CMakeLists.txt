cmake_minimum_required(VERSION 2.8.3)
project(drivers_ax12)


add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  message_generation
  message_runtime
  roscpp
  std_msgs
  drivers_port_finder
)

add_action_files(
  DIRECTORY action
  FILES
  AngleCommand.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES drivers_ax12
  CATKIN_DEPENDS actionlib actionlib_msgs roscpp std_msgs drivers_port_finder
  #DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

find_library(DXL_LIB dxl ${PROJECT_SOURCE_DIR}/lib)

add_executable(ax12_node src/ax12_node.cpp src/ax12_server.cpp)

add_dependencies(ax12_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(
  ax12_node
  ${DXL_LIB}
  ${catkin_LIBRARIES}
)

