cmake_minimum_required(VERSION 2.8)
get_filename_component(PROJECT ${CMAKE_CURRENT_LIST_DIR} NAME)
project(${PROJECT})
# cmake_policy(SET CMP0046 OLD)

#set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_BUILD_TYPE "Debug")

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/../../cmake)
include(../../cmake/ConfigureCompiler.cmake)
include(../../cmake/ConfigureOpenCV.cmake)

add_executable(${PROJECT} main.cpp)
target_link_libraries(${PROJECT} ${LIBRARIES})
file(COPY "input.png" DESTINATION ${PROJECT_BINARY_DIR})
