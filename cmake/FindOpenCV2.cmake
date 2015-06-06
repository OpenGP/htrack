# To find OpenCV 2 library visit http://opencv.willowgarage.com/wiki/
#
# The follwoing variables are optionally searched for defaults
#  OpenCV2_ROOT_DIR:                   Base directory of OpenCV 2 tree to use.
#
# The following are set after configuration is done:
#  OpenCV2_FOUND
#  OpenCV2_INCLUDE_DIRS
#  OpenCV2_LIBRARIES
#
# $Id: $
#  
# Balazs [2011-01-18]:
# - Created from scratch for the reorganized OpenCV 2 structure introduced at version 2.2
# Jbohren [2011-06-10]:
# - Added OpenCV_ROOT_DIR for UNIX platforms & additional opencv include dir
# jmorrison [2013-11-14]:
# - Added flag to disable GPU requirement (NO_OPENCV_GPU)
# 
# This file should be removed when CMake will provide an equivalent

#--- Select exactly ONE OpenCV 2 base directory to avoid mixing different version headers and libs
find_path(OpenCV2_ROOT_INC_DIR NAMES opencv2/opencv.hpp
    PATHS
        #--- WINDOWS
        C:/Developer/include                # Windows
        "$ENV{OpenCV_ROOT_DIR}/include"     # *NIX: custom install
        /usr/local/include                  # Linux: default dir by CMake
        /usr/include                        # Linux
        /opt/local/include                  # OS X: default MacPorts location
        NO_DEFAULT_PATH)

#--- DEBUG
#message(STATUS "OpenCV2_ROOT_INC_DIR: ${OpenCV2_ROOT_INC_DIR}")

#--- OBSOLETE
# Get parent of OpenCV2_ROOT_INC_DIR. We do this as it is more
# reliable than finding include/opencv2/opencv.hpp directly.
#GET_FILENAME_COMPONENT(OpenCV2_ROOT_DIR ${OpenCV2_ROOT_INC_DIR} PATH)
#message(STATUS "OpenCV2_ROOT_DIR: ${OpenCV2_ROOT_DIR}")

find_path(OpenCV2_CORE_INCLUDE_DIR       NAMES core.hpp         PATHS "${OpenCV2_ROOT_INC_DIR}/opencv2/core")
find_path(OpenCV2_IMGPROC_INCLUDE_DIR    NAMES imgproc.hpp      PATHS "${OpenCV2_ROOT_INC_DIR}/opencv2/imgproc")
find_path(OpenCV2_CONTRIB_INCLUDE_DIR    NAMES contrib.hpp      PATHS "${OpenCV2_ROOT_INC_DIR}/opencv2/contrib")
find_path(OpenCV2_HIGHGUI_INCLUDE_DIR    NAMES highgui.hpp      PATHS "${OpenCV2_ROOT_INC_DIR}/opencv2/highgui")
find_path(OpenCV2_FLANN_INCLUDE_DIR      NAMES flann.hpp        PATHS "${OpenCV2_ROOT_INC_DIR}/opencv2/flann")

set(OpenCV2_INCLUDE_DIRS
    ${OpenCV2_ROOT_INC_DIR}
    ${OpenCV2_ROOT_INC_DIR}/opencv2
    ${OpenCV2_CORE_INCLUDE_DIR}
    ${OpenCV2_IMGPROC_INCLUDE_DIR}
    ${OpenCV2_CONTRIB_INCLUDE_DIR}
    ${OpenCV2_HIGHGUI_INCLUDE_DIR}
    ${OpenCV2_FLANN_INCLUDE_DIR})

# absolute path to all libraries 
# set(OPENCV2_LIBRARY_SEARCH_PATHS "${OpenCV2_ROOT_DIR}/lib")

#--- Specify where DLL is searched for
#message(STATUS "OPENCV2_LIBRARY_SEARCH_PATHS ${OPENCV2_LIBRARY_SEARCH_PATHS}")
list(APPEND OPENCV2_LIBRARY_SEARCH_PATHS $ENV{OpenCV_ROOT_DIR})
list(APPEND OPENCV2_LIBRARY_SEARCH_PATHS "C:/Developer/lib")
list(APPEND OPENCV2_LIBRARY_SEARCH_PATHS "/usr/local/lib")
list(APPEND OPENCV2_LIBRARY_SEARCH_PATHS "/opt/local/lib")
list(APPEND OPENCV2_LIBRARY_SEARCH_PATHS "/usr/lib")


#--- FIND RELEASE LIBRARIES
find_library(OpenCV2_CORE_LIBRARY_REL       NAMES opencv_core opencv_core230 opencv_core220 opencv_core2410                         PATHS ${OPENCV2_LIBRARY_SEARCH_PATHS})
find_library(OpenCV2_IMGPROC_LIBRARY_REL    NAMES opencv_imgproc opencv_imgproc230 opencv_imgproc220 opencv_imgproc2410             PATHS ${OPENCV2_LIBRARY_SEARCH_PATHS})
find_library(OpenCV2_CONTRIB_LIBRARY_REL    NAMES opencv_contrib opencv_contrib230 opencv_contrib220 opencv_contrib2410             PATHS ${OPENCV2_LIBRARY_SEARCH_PATHS})
find_library(OpenCV2_HIGHGUI_LIBRARY_REL    NAMES opencv_highgui opencv_highgui230 opencv_highgui220 opencv_highgui2410             PATHS ${OPENCV2_LIBRARY_SEARCH_PATHS})
list(APPEND OpenCV2_LIBRARIES_REL ${OpenCV2_CORE_LIBRARY_REL})
list(APPEND OpenCV2_LIBRARIES_REL ${OpenCV2_IMGPROC_LIBRARY_REL})
list(APPEND OpenCV2_LIBRARIES_REL ${OpenCV2_CONTRIB_LIBRARY_REL})
list(APPEND OpenCV2_LIBRARIES_REL ${OpenCV2_HIGHGUI_LIBRARY_REL})

#--- FIND DEBUG LIBRARIES
if(WIN32)
    find_library(OpenCV2_CORE_LIBRARY_DEB       NAMES opencv_cored opencv_core230d opencv_core220d opencv_core2410d                     PATHS ${OPENCV2_LIBRARY_SEARCH_PATHS})
    find_library(OpenCV2_IMGPROC_LIBRARY_DEB    NAMES opencv_imgprocd opencv_imgproc230d opencv_imgproc220d opencv_imgproc2410d         PATHS ${OPENCV2_LIBRARY_SEARCH_PATHS})
    find_library(OpenCV2_CONTRIB_LIBRARY_DEB    NAMES opencv_contribd opencv_contrib230d opencv_contrib220d opencv_contrib2410d         PATHS ${OPENCV2_LIBRARY_SEARCH_PATHS})
    find_library(OpenCV2_HIGHGUI_LIBRARY_DEB    NAMES opencv_highguid opencv_highgui230d opencv_highgui220d opencv_highgui2410d         PATHS ${OPENCV2_LIBRARY_SEARCH_PATHS})
    list(APPEND OpenCV2_LIBRARIES_DEB ${OpenCV2_CORE_LIBRARY_DEB})
    list(APPEND OpenCV2_LIBRARIES_DEB ${OpenCV2_IMGPROC_LIBRARY_DEB})
    list(APPEND OpenCV2_LIBRARIES_DEB ${OpenCV2_CONTRIB_LIBRARY_DEB})
    list(APPEND OpenCV2_LIBRARIES_DEB ${OpenCV2_HIGHGUI_LIBRARY_DEB})
endif()

#--- Setup cross-config libraries
set(OpenCV2_LIBRARIES "")
if(WIN32)
    list(APPEND OpenCV2_LIBRARIES optimized ${OpenCV2_CORE_LIBRARY_REL}    debug ${OpenCV2_CORE_LIBRARY_DEB})
    list(APPEND OpenCV2_LIBRARIES optimized ${OpenCV2_IMGPROC_LIBRARY_REL} debug ${OpenCV2_IMGPROC_LIBRARY_DEB})
    list(APPEND OpenCV2_LIBRARIES optimized ${OpenCV2_CONTRIB_LIBRARY_REL} debug ${OpenCV2_CONTRIB_LIBRARY_DEB})
    list(APPEND OpenCV2_LIBRARIES optimized ${OpenCV2_HIGHGUI_LIBRARY_REL} debug ${OpenCV2_HIGHGUI_LIBRARY_DEB})
else()
    list(APPEND OpenCV2_LIBRARIES ${OpenCV2_CORE_LIBRARY_REL}   )
    list(APPEND OpenCV2_LIBRARIES ${OpenCV2_IMGPROC_LIBRARY_REL})
    list(APPEND OpenCV2_LIBRARIES ${OpenCV2_CONTRIB_LIBRARY_REL})
    list(APPEND OpenCV2_LIBRARIES ${OpenCV2_HIGHGUI_LIBRARY_REL})
endif()

#--- Verifies everything (include) was found
set(OpenCV2_FOUND ON)
FOREACH(NAME ${OpenCV2_INCLUDE_DIRS})
    IF(NOT EXISTS ${NAME})
        message(WARNING "Could not find: ${NAME}")
        set(OpenCV2_FOUND OFF)
    endif(NOT EXISTS ${NAME})
ENDFOREACH(NAME)

#--- Verifies everything (release lib) was found
FOREACH(NAME ${OpenCV2_LIBRARIES_REL})
    IF(NOT EXISTS ${NAME})
        message(WARNING "Could not find: ${NAME}")
        set(OpenCV2_FOUND OFF)
    endif(NOT EXISTS ${NAME})
 ENDFOREACH()

#--- Verifies everything (debug lib) was found
FOREACH(NAME ${OpenCV2_LIBRARIES_DEB})
    IF(NOT EXISTS ${NAME})
        message(WARNING "Could not find: ${NAME}")
        set(OpenCV2_FOUND OFF)
    endif(NOT EXISTS ${NAME})
ENDFOREACH()

#--- Display help message
IF(NOT OpenCV2_FOUND)
    IF(OpenCV2_FIND_REQUIRED)
        MESSAGE(FATAL_ERROR "OpenCV 2 not found.")
    else()
        MESSAGE(STATUS "OpenCV 2 not found.")
    endif()
endif()
