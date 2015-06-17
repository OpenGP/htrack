if(NOT WIN32)
    return()
endif()

cmake_minimum_required(VERSION 2.8)
get_filename_component(PROJECT ${CMAKE_CURRENT_LIST_DIR} NAME)
project(${PROJECT})

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/../../cmake)
include(../../cmake/ConfigureCompiler.cmake)
include(../../cmake/ConfigureOpenNI.cmake)
include(../../cmake/ConfigureOpenCV.cmake)
include(../../cmake/ConfigureQt.cmake)
include(../../cmake/ConfigureEigen.cmake)
include(../../cmake/ConfigureRealSense.cmake)

set(CMAKE_BUILD_TYPE "Debug")
INCLUDE_DIRECTORIES(../../)
list(APPEND EXTRA_SOURCES ../../tracker/Data/Camera.cpp)
list(APPEND EXTRA_SOURCES ../../tracker/Sensor/Sensor_realsense.cpp)

add_executable(${PROJECT} main.cpp ${EXTRA_SOURCES})
target_link_libraries(${PROJECT} ${LIBRARIES})
