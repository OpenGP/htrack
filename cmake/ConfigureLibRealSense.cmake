add_compile_options(-std=c++11 -fPIC -lusb-1.0 -lpthread )

# libusb-1.0
find_package(Threads REQUIRED)
find_package(PkgConfig)
pkg_check_modules(PC_LIBUSB REQUIRED libusb-1.0)
include_directories( ${LIBUSB_1_INCLUDE_DIRS})

## GLFW
#find_package(PkgConfig REQUIRED)
#pkg_search_module(GLFW REQUIRED glfw3)
#include_directories(${GLFW_INCLUDE_DIRS})
#LIST(APPEND LIBRARIES ${GLFW_LIBRARIES})


# librealsense
set(LIBREALSENSE_INCLUDE_DIR "/home/raghu/Desktop/librealsense/include")
set(LIBREALSENSE_LIBRARY "/usr/local/lib/librealsense.so")
include_directories(${LIBREALSENSE_INCLUDE_DIR})
list(APPEND LIBRARIES ${LIBREALSENSE_LIBRARY})


#--- Mark as available
add_definitions(-DHAS_LIBREALSENSE)
