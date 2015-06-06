find_package(OpenCV2 REQUIRED)

if(OpenCV2_FOUND)
    include_directories(${OpenCV2_INCLUDE_DIRS})
    LIST(APPEND LIBRARIES ${OpenCV2_LIBRARIES})
    add_definitions(-DWITH_OPENCV)
else()
    message(FATAL_ERROR "OpenCV required")
endif()

#message(STATUS "OpenCV2_LIBRARIES: ${OpenCV2_LIBRARIES}")
#message(STATUS "OpenCV2_INCLUDE_DIRS: ${OpenCV2_INCLUDE_DIRS}")

#--- (OBSOLETE) find it with PkgConfig
#find_package(PkgConfig REQUIRED)
#pkg_check_modules(OPENCV REQUIRED opencv)
#list(APPEND LIBRARIES ${OPENCV_LDFLAGS})
#include_directories(${OPENCV_INCLUDE_DIRS})

