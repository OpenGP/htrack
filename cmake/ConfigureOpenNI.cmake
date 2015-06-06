#--- OpenNI2
find_package(OpenNI2)
if(OPENNI2_FOUND)
    include_directories(${OPENNI2_INCLUDE_DIR})
    list(APPEND LIBRARIES ${OPENNI2_LIBRARY})
    add_definitions(-DWITH_OPENNI)
else()
    message(STATUS "WARNING: OpenNI2 not found!")
endif()

#message(STATUS "OpenNI2: ${OPENNI2_LIBRARY}")
#message(STATUS "OpenNI2: ${OPENNI2_INCLUDE_DIR}")
