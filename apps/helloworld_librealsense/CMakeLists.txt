# https://github.com/IntelRealSense/librealsense/issues/62

cmake_minimum_required(VERSION 2.8)
get_filename_component(PROJECT ${CMAKE_CURRENT_LIST_DIR} NAME)
project(${PROJECT})
add_compile_options(-std=c++11 -fPIC -lusb-1.0 -lpthread )


# libusb-1.0
find_package(Threads REQUIRED)
find_package(PkgConfig)
pkg_check_modules(PC_LIBUSB REQUIRED libusb-1.0)
include_directories( ${LIBUSB_1_INCLUDE_DIRS})

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/../../cmake)
# include(../../cmake/ConfigureOpenCV.cmake)
include(../../cmake/ConfigureOpenGL.cmake)

# GLUT
#find_package(GLUT REQUIRED)
#include_directories(${GLUT_INCLUDE_DIRS})
#LIST(APPEND LIBRARIES ${GLUT_LIBRARY})

# GLFW
find_package(PkgConfig REQUIRED)
pkg_search_module(GLFW REQUIRED glfw3)
include_directories(${GLFW_INCLUDE_DIRS})
LIST(APPEND LIBRARIES ${GLFW_LIBRARIES})



# librealsense
set(LIBREALSENSE_INCLUDE_DIR "/home/raghu/Desktop/librealsense/include")
set(LIBREALSENSE_LIBRARY "/usr/local/lib/librealsense.so")
include_directories(${LIBREALSENSE_INCLUDE_DIR})
list(APPEND LIBRARIES ${LIBREALSENSE_LIBRARY})

aux_source_directory(. SRC_LIST)
add_executable(${PROJECT_NAME} ${SRC_LIST})
target_link_libraries( ${PROJECT_NAME} ${LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} )
