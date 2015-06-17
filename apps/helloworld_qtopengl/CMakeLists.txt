cmake_minimum_required(VERSION 2.8)
get_filename_component(PROJECT ${CMAKE_CURRENT_LIST_DIR} NAME)
project(${PROJECT})

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/../../cmake)
include(../../cmake/ConfigureCompiler.cmake)
include(../../cmake/ConfigureQt.cmake)
include(../../cmake/ConfigureOpenGL.cmake)

add_executable(${PROJECT} main.cpp)
target_link_libraries(${PROJECT} ${LIBRARIES})
