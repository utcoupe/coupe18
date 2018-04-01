cmake_minimum_required(VERSION 2.8.3)
project(recognition_enemy_racker)

find_package(catkin REQUIRED COMPONENTS
    ai_game_status
    recognition_objects_classifier
)

catkin_package(
    CATKIN_DEPENDS
    ai_game_status
    recognition_objects_classifier
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)