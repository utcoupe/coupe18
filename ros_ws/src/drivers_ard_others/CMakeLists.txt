cmake_minimum_required(VERSION 2.8.3)
project(drivers_ard_others)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    geometry_msgs
    std_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
    FILES
    BeltRange.msg
    Move.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    geometry_msgs # std_msgs
)

catkin_package(
    CATKIN_DEPENDS 
    geometry_msgs 
    std_msgs
    message_runtime
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)