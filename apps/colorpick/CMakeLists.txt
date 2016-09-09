#--- This project needs this extra library
#include(../../cmake/ConfigureQGLViewer.cmake)

INCLUDE_DIRECTORIES(../../) #< #include "tracker/..."
file(GLOB_RECURSE SRC "*.cpp")
file(GLOB_RECURSE HDR "*.h")
add_executable(colorpick ${HDR} ${SRC})
target_link_libraries(colorpick htracksdk ${LIBRARIES})
