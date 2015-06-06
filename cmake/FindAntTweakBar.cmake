#
# Try to find AntTweakBar library and include path.
# Once done this will define
#
# ANTTWEAKBAR_FOUND
# ANTTWEAKBAR_INCLUDE_DIR
# ANTTWEAKBAR_LIBRARY
#

FIND_PATH(ANTTWEAKBAR_INCLUDE_DIR AntTweakBar.h PATHS
    $ENV{ANTTWEAKBAR_ROOT}/include
    #--- External folder
    ${CMAKE_CURRENT_LIST_DIR}/../external/AntTweakBar/include/
    #--- Windows
    C:/Developer/include/AntTweakBar
    #--- Mac
    /usr/local/include
    /usr/X11/include
    /usr/include
    NO_DEFAULT_PATH
    DOC "The directory where AntTweakBar.h resides")

if(WIN32)
    set(BITS "64")
endif()

FIND_LIBRARY( ANTTWEAKBAR_LIBRARY AntTweakBar${BITS} PATHS
    $ENV{ANTTWEAKBAR_ROOT}/lib
    #--- Windows
    C:/Developer/lib
    #---
    /usr/local
    /usr/X11
    /usr
    PATH_SUFFIXES a lib64 lib dylib NO_DEFAULT_PATH
    DOC "The AntTweakBar library")

if(ANTTWEAKBAR_INCLUDE_DIR AND ANTTWEAKBAR_LIBRARY)
    message(STATUS "Found ANTTWEAKBAR: ${ANTTWEAKBAR_LIBRARY}")
    set(ANTTWEAKBAR_FOUND TRUE)
endif()

#--- DEBUG
#message(STATUS "ANTTWEAKBAR_INCLUDE_DIR: ${ANTTWEAKBAR_INCLUDE_DIR}")
#message(STATUS "ANTTWEAKBAR_LIBRARY: ${ANTTWEAKBAR_LIBRARY}")
