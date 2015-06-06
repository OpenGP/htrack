#--- This project needs this extra library
include(../../cmake/ConfigureQGLViewer.cmake)

INCLUDE_DIRECTORIES(../../) #< #include "tracker/..."
qt5_wrap_cpp(MOCS_HDRS Main_window.h OpenGL_viewer.h)
file(GLOB_RECURSE SRC "*.cpp")
file(GLOB_RECURSE HDR "*.h")
add_executable(htrack_qt ${HDR} ${SRC} ${MOCS_HDRS})
target_link_libraries(htrack_qt htracksdk ${LIBRARIES})
