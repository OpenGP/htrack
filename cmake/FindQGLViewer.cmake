# - Try to find QGLViewer
# Once done this will define
#
#  QGLVIEWER_FOUND - system has QGLViewer
#  QGLVIEWER_INCLUDE_DIR - the QGLViewer include directory
#  QGLVIEWER_LIBRARIES - Link these to use QGLViewer
#  QGLVIEWER_DEFINITIONS - Compiler switches required for using QGLViewer
#

find_path(QGLVIEWER_INCLUDE_DIR 
    NAMES QGLViewer/qglviewer.h
    PATHS
        #--- Windows
        C:/Developer/include
        #--- Mac
        /Library/Frameworks
        #--- Linux
        /usr/include
        /usr/local/include
        #--- Environment
        ENV QGLVIEWERROOT
    PATH_SUFFIXES Headers)

find_library(QGLVIEWER_LIBRARY_RELEASE
    NAMES QGLViewer QGLViewer2 QGLViewer2
    HINTS
        #--- Windows
        C:/Developer/lib
        #--- Linux
        /usr/lib
        /usr/local/lib
        #--- Mac
        /Library/Frameworks
        #--- Environment
        ENV QGLVIEWERROOT
        ENV LD_LIBRARY_PATH
        ENV LIBRARY_PATH
    PATH_SUFFIXES
        QGLViewer
        QGLViewer/release)

find_library(QGLVIEWER_LIBRARY_DEBUG
    NAMES
        dqglviewer
        dQGLViewer
        dQGLViewer2
        QGLViewerd2
    PATHS
        #--- Windows
        C:/Developer/lib
        #--- Linux
        /usr/lib
        /usr/local/lib
        #--- Mac
        /Library/Frameworks
        #--- Environment
        ENV QGLVIEWERROOT
        ENV LD_LIBRARY_PATH
        ENV LIBRARY_PATH
    PATH_SUFFIXES
        QGLViewer
        QGLViewer/debug)

if(QGLVIEWER_LIBRARY_RELEASE)
    if(QGLVIEWER_LIBRARY_DEBUG)
        set(QGLVIEWER_LIBRARIES_ optimized ${QGLVIEWER_LIBRARY_RELEASE} debug ${QGLVIEWER_LIBRARY_DEBUG})
    else()
        set(QGLVIEWER_LIBRARIES_ ${QGLVIEWER_LIBRARY_RELEASE})
    endif()
    set(QGLVIEWER_LIBRARIES ${QGLVIEWER_LIBRARIES_} CACHE FILEPATH "The QGLViewer library")
endif()

#message(STATUS "QGLVIEWER_INCLUDE_DIR: ${QGLVIEWER_INCLUDE_DIR}")
#message(STATUS "QGLVIEWER_LIBRARIES: ${QGLVIEWER_LIBRARIES}")

IF(QGLVIEWER_INCLUDE_DIR AND QGLVIEWER_LIBRARIES)
   SET(QGLVIEWER_FOUND TRUE)
ENDIF()

IF(QGLVIEWER_FOUND)
    IF(NOT CMAKE_FIND_QUIETLY)
        MESSAGE(STATUS "Found QGLViewer: ${QGLVIEWER_LIBRARIES}")
    ENDIF()
ELSE()
    IF(QGLViewer_FIND_REQUIRED)
        MESSAGE(FATAL_ERROR "Could not find QGLViewer")
    ENDIF()
ENDIF()

