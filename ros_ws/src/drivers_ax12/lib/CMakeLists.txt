cmake_minimum_required (VERSION 2.8.3)
include_directories(include)
add_library(dxl src/dxl_hal.c src/dynamixel.c)
target_include_directories(dxl PUBLIC include)


