PROJECT(hokuyo)
cmake_minimum_required(VERSION 2.8)

FIND_PACKAGE(SDL REQUIRED)

SET(CMAKE_C_FLAGS "-std=gnu99 -Wall -Werror")

SET(EXECUTABLE_OUTPUT_PATH $ENV{UTCOUPE_WORKSPACE}/bin/)

file(GLOB_RECURSE SRC_FILES	src/*)

add_executable(${PROJECT_NAME}	${SRC_FILES})

#TODO c_urg by find_package
INCLUDE_DIRECTORIES(${SDL_INCLUDE_DIR} /usr/local/include/c_urg)

#TODO urg_libraries instead of each urg library by hand
target_link_libraries(${PROJECT_NAME} c_urg c_urg_connection c_urg_system SDL m )
