# Locate the GLFW library (version 2.0)
# This module defines the following variables:
# GLFW_LIBRARIES, the name of the library;
# GLFW_INCLUDE_DIRS, where to find GLFW include files.
# GLFW_FOUND, true if library path was resolved
#
# Usage example to compile an "executable" target to the glfw library:
#
# FIND_PACKAGE (GLFW REQUIRED)
# INCLUDE_DIRECTORIES (${GLFW_INCLUDE_DIRS})
# ADD_EXECUTABLE (executable ${YOUR_EXECUTABLE_SRCS})
# TARGET_LINK_LIBRARIES (executable ${GLFW_LIBRARIES})
#
# TODO:
# Lookup for windows
# Allow the user to select to link to a shared library or to a static library.
#
# SEE:
# - https://raw.github.com/progschj/OpenGL-Examples/master/cmake_modules/FindGLFW.cmake
# 

FIND_PATH( GLFW_INCLUDE_DIRS GL/glfw.h
    $ENV{GLFWDIR}/include
    /usr/local/include
    /usr/local/X11R6/include
    /usr/X11R6/include
    /usr/X11/include
    /usr/include/X11
    /usr/include
    /opt/X11/include
    /opt/include 
	#--- FOR WINDOWS 
	C:/Developer/include
    #--- Deployed 
	${CMAKE_SOURCE_DIR}/external/glfw/include)
	
message(STATUS "GLFW_INCLUDE_DIRS: ${GLFW_INCLUDE_DIRS}")
	
FIND_LIBRARY( GLFW_LIBRARIES NAMES glfw PATHS 
    $ENV{GLFWDIR}/lib
    /usr/local/lib
    /usr/local/X11R6/lib
    /usr/X11R6/lib
    /usr/X11/lib
    /usr/lib/X11
    /usr/lib
    /opt/X11/lib
    /opt/lib 	
	#--- FOR WINDOWS 
	C:/Developer/lib
	#--- Deployed
    ${CMAKE_SOURCE_DIR}/external/glfw/lib)

SET(GLFW_FOUND "NO")
IF(GLFW_LIBRARIES AND GLFW_INCLUDE_DIRS)
    SET(GLFW_FOUND "YES")
ENDIF(GLFW_LIBRARIES AND GLFW_INCLUDE_DIRS)
