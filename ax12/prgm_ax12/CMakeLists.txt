# Nous voulons un cmake "récent" pour utiliser les dernières fonctionnalités
cmake_minimum_required(VERSION 2.8)

# Notre projet est étiqueté hello
project(ax12)
set(EXECUTABLE_OUTPUT_PATH bin/${CMAKE_BUILD_TYPE})
SET(CMAKE_CXX_FLAGS "-std=c++11")

include_directories(../lib/include)

link_directories(../lib)

file(GLOB_RECURSE source_files src/*.cpp )

if(DEFINED ENV{UTCOUPE_WORKSPACE})
    SET(EXECUTABLE_OUTPUT_PATH $ENV{UTCOUPE_WORKSPACE}/bin/)
else()
	message("[PATHFINDING] No UTCOUPE_WORKSPACE env variable set... Add exec file in pathfinding/bin folder.")
    SET(EXECUTABLE_OUTPUT_PATH ../bin/)
endif()

#Configuration de l'exécutable
add_executable(
        ax12
        ${source_files}
)

#target_compile_options(ax12 PUBLIC "-lpthread ")
#Configuration de l'édition de liens

target_link_libraries(
        ax12
        libdxl.a
        "-lpthread"
)
