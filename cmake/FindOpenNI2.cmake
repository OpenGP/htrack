# OPENNI2_INCLUDE_DIR
# OPENNI2_LIBRARY
# OPENNI2_FOUND

find_path(OPENNI2_INCLUDE_DIR "OpenNI.h"
    PATHS
        #--- MAC
        /usr/include/ni2
        /usr/local/include/ni2
        #--- WINDOWS
        C:/Developer/include/openni
        #--- LINUX (self deployed)
        ${CMAKE_SOURCE_DIR}/openni/include
        ${CMAKE_SOURCE_DIR}/../openni/include
    DOC "OpenNI c++ interface header")

find_library(OPENNI2_LIBRARY "OpenNI2"
    PATHS
        #--- MAC
        /usr/lib/ni2
        /usr/local/lib/ni2
        #--- LINUX (self deployed)
        /usr/lib
        #--- WINDOWS
        C:/Developer/lib
        C:/Program Files/OpenNI2/Lib
        ${CMAKE_SOURCE_DIR}/openni/lib
        ${CMAKE_SOURCE_DIR}/../openni/lib
    DOC "OpenNI2 library")

#message(STATUS "OPENNI2_LIBRARY: ${OPENNI2_LIBRARY}")
#message(STATUS "OPENNI2_INCLUDE_DIR: ${OPENNI2_INCLUDE_DIR}")

if(OPENNI2_INCLUDE_DIR AND OPENNI2_LIBRARY)
    set(OPENNI2_FOUND TRUE)
endif()

#--- Notifications
if(OPENNI2_FOUND AND NOT OpenNI2__FIND_QUIETLY)
    message(STATUS "Found OpenNI: ${OPENNI2_LIBRARY}")
else()
    if(OpenNI2_FIND_REQUIRED)
        message(STATUS OPENNI2_INCLUDE_DIR: ${OPENNI2_INCLUDE_DIR})
        message(STATUS OPENNI2_LIBRARY:     ${OPENNI2_LIBRARY})
        message(FATAL_ERROR "Could not find OpenNI2")
    endif()
endif()
