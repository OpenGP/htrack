cmake_minimum_required(VERSION 2.8)
get_filename_component(PROJECT ${CMAKE_CURRENT_LIST_DIR} NAME)
project(${PROJECT})
#cmake_policy(SET CMP0020 OLD)

#set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_BUILD_TYPE "Debug")

#--- Where to find the FindPackageName.cmake
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/../../cmake)
include(../../cmake/ConfigureCompiler.cmake)
include(../../cmake/ConfigureQt.cmake)
include(../../cmake/ConfigureOpenGL.cmake)
include(../../cmake/ConfigureQGLViewer.cmake)

#--- Create exe and link
add_executable(${PROJECT} main.cpp)
target_link_libraries(${PROJECT} ${LIBRARIES})
