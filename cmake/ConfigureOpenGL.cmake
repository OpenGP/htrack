#--- OpenGL
find_package(OpenGL REQUIRED)
if(NOT OPENGL_FOUND)
    message(ERROR " OPENGL not found!")
endif(NOT OPENGL_FOUND)

include_directories(${OpenGL_INCLUDE_DIRS})
link_directories(${OpenGL_LIBRARY_DIRS})
add_definitions(${OpenGL_DEFINITIONS})
LIST(APPEND LIBRARIES ${OPENGL_LIBRARIES})

if(UNIX)
    LIST(APPEND LIBRARIES ${OPENGL_glu_LIBRARY})
endif()
