find_package(QGLViewer) #<<< Optional
if(QGLVIEWER_FOUND)
    include_directories(${QGLVIEWER_INCLUDE_DIR})
    add_definitions(-DQGLVIEWER)
    LIST(APPEND LIBRARIES ${QGLVIEWER_LIBRARIES})
    add_definitions(-DWITH_QGLVIEWER)
    #message(STATUS "INCLUDE DIR" ${QGLVIEWER_INCLUDE_DIR})
endif()
