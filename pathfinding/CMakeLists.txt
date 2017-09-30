#Configuration du projet
cmake_minimum_required(VERSION 2.8)
project(pathfinding)

FIND_PACKAGE(Boost REQUIRED)
SET(CMAKE_CXX_FLAGS "-std=c++11 -Wall -Werror")
SET(TCLAP_INCLUDE_DIR ../libs/tclap-1.2.1/include)

if(DEFINED ENV{UTCOUPE_WORKSPACE})
    SET(EXECUTABLE_OUTPUT_PATH $ENV{UTCOUPE_WORKSPACE}/bin/)
else()
	message("[PATHFINDING] No UTCOUPE_WORKSPACE env variable set... Add exec file in pathfinding/bin folder.")
    SET(EXECUTABLE_OUTPUT_PATH ../bin/)
endif()

file(GLOB_RECURSE LIBRARIES_SOURCES src/lib/*)
file(GLOB_RECURSE SOURCE_FILES	src/main.cpp)
include_directories(${TCLAP_INCLUDE_DIR})

add_executable(${PROJECT_NAME} ${LIBRARIES_SOURCES} ${SOURCE_FILES})

# Test_map binary

SET(TEST_MAP_EXEC_NAME test_map)
file(GLOB_RECURSE TEST_SOURCE_FILES	src/test/map_test.cpp)
add_executable(${TEST_MAP_EXEC_NAME} ${LIBRARIES_SOURCES} ${TEST_SOURCE_FILES})
