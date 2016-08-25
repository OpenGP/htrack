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

# OpenGL
find_package(OpenGL REQUIRED)
include_directories(${OpenGL_INCLUDE_DIRS})
link_directories(${OpenGL_LIBRARY_DIRS})
add_definitions(${OpenGL_DEFINITIONS})
if(NOT OPENGL_FOUND)
    message(ERROR " OPENGL not found!")
endif(NOT OPENGL_FOUND)

# GLUT
find_package(GLUT REQUIRED)
include_directories(${GLUT_INCLUDE_DIRS} )

# GLFW
find_package(PkgConfig REQUIRED)
 pkg_search_module(GLFW REQUIRED glfw3)
include_directories(${GLFW_INCLUDE_DIRS})

# OpenCV
find_package( OpenCV REQUIRED )

# librealsense
include_directories("/home/raghu/Desktop/librealsense/include")
link_directories("/usr/local/lib")
set(LIBS ${LIBS} realsense)

link_directories( ${realsense_LIBRARY})
include_directories( ${realsense_INCLUDE_DIR})


aux_source_directory(. SRC_LIST)
add_executable(${PROJECT_NAME} ${SRC_LIST})
target_link_libraries( ${PROJECT_NAME}
${LIBS} ${OpenCV_LIBS} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} ${GLFW_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} )
