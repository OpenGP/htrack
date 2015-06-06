INCLUDE_DIRECTORIES(../../) #< #include "tracker/..."
file(GLOB_RECURSE SRC "*.cpp")
file(GLOB_RECURSE HDR "*.h")
add_executable(htrack_atb ${HDR} ${SRC})
target_link_libraries(htrack_atb htracksdk)

