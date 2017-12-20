cmake_minimum_required(VERSION 2.8.3)
project(navigation_collisions)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    rospy
    std_msgs
    geometry_msgs
    memory_map
    message_generation
)

##############
## Messages ##
##############

# Generate messages
add_message_files(
    FILES
    PredictedCollision.msg
)

# Generate added messages and services with any dependencies listed here
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
    CATKIN_DEPENDS rospy std_msgs message_runtime
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
