FIND_PATH( GLEW_INCLUDE_DIRS GL/glew.h
    $ENV{GLEWDIR}/include
    /usr/local/include
    /usr/local/X11R6/include
    /usr/X11R6/include
    /usr/X11/include
    /usr/include/X11
    /usr/include
    /opt/X11/include
    /opt/include 
    # FOR WINDOWS
    C:/Developer/include
    ${CMAKE_SOURCE_DIR}/external/glew/include)

if(WIN32)
    SET(GLEWLIBNAME glew32s) # static
#    SET(GLEWLIBNAME glew32) # dyn
else()
    # IMPORTANT: uppercase otherwise problem on linux
    SET(GLEWLIBNAME GLEW)
endif() 

FIND_LIBRARY( GLEW_LIBRARIES NAMES ${GLEWLIBNAME} PATHS
    $ENV{GLEWDIR}/lib
    /usr/local/lib
    /usr/local/X11R6/lib
    /usr/X11R6/lib
    /usr/X11/lib
    /usr/lib/X11
    /usr/lib
    /opt/X11/lib
    /opt/lib
    # FOR UBUNTU 12.04 LTS
    /usr/lib/x86_64-linux-gnu    
    # FOR WINDOWS 
    C:/Developer/lib
    ${CMAKE_SOURCE_DIR}/external/glew/lib)


SET(GLEW_FOUND "NO")
IF(GLEW_LIBRARIES AND GLEW_INCLUDE_DIRS)
    SET(GLEW_FOUND "YES")
ENDIF()

#message(STATUS "GLEW_LIBRARIES: ${GLEW_LIBRARIES}")
#message(STATUS "GLEW_INCLUDE_DIRS: ${GLEW_INCLUDE_DIRS}")
