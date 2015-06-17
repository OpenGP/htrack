cmake_minimum_required(VERSION 2.8)
get_filename_component(PROJECTNAME ${CMAKE_CURRENT_LIST_DIR} NAME)
project(${PROJECTNAME})

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/../../cmake)
include(../../cmake/ConfigureCompiler.cmake)
include(../../cmake/ConfigureQt.cmake)
include(../../cmake/ConfigureOpenGL.cmake)
include(../../cmake/ConfigureGLEW.cmake)
include(../../cmake/ConfigureCUDA.cmake)
include(../../cmake/ConfigureEigen.cmake)
include(../../cmake/ConfigureOpenGL.cmake)

set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_BUILD_TYPE "Debug")

include_directories(../../) #< #include "tracker/..."
file(GLOB SOURCES "*.h")
file(GLOB HEADERS "*.cpp")
add_executable(${PROJECTNAME} ${SOURCES} ${HEADERS})
target_link_libraries(${PROJECTNAME} ${LIBRARIES})
