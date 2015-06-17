cmake_minimum_required(VERSION 2.8)
get_filename_component(PROJECT ${CMAKE_CURRENT_LIST_DIR} NAME)
project(${PROJECT})

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/../../cmake)
include(../../cmake/ConfigureCompiler.cmake)
include(../../cmake/ConfigureQt.cmake)
include(../../cmake/ConfigureOpenGL.cmake)
include(../../cmake/ConfigureAntTweakBar.cmake)

INCLUDE_DIRECTORIES(../../)
#set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_BUILD_TYPE "Debug")

list(APPEND SOURCES ../../tracker/TwSettings.cpp)
add_executable(${PROJECT} main.cpp ${SOURCES})
target_link_libraries(${PROJECT} ${LIBRARIES})
#set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR})
