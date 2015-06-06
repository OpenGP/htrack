#--- TODO: fully decouple library in subdirectory
# http://stackoverflow.com/questions/7819647/cmake-build-library-and-link-against-it

#--- Add the "tracker" folder (recursively)
file(GLOB_RECURSE SRC_TRACKER "*.cpp")
file(GLOB_RECURSE HDR_TRACKER "*.h")
list(APPEND SOURCES ${SRC_TRACKER})
list(APPEND HEADERS ${HDR_TRACKER})

#--- Makes DebugRenderer visible
include_directories(OpenGL)
include_directories(OpenGL/DebugRenderer)
include_directories(OpenGL/CylindersRenderer)
include_directories(OpenGL/QuadRenderer)
include_directories(OpenGL/KinectDataRenderer)

#--- Shaders
# Dummy Target to have shaders appear in project
file(GLOB_RECURSE SHADER_SOURCES "tracker/OpenGL/*.glsl")
add_custom_target(shaders SOURCES ${SHADER_SOURCES})
# @see note on Q_INIT_RESOURCE http://qt-project.org/wiki/QtResources
qt5_add_resources(SHADERS_QRC OpenGL/shaders.qrc)

#--- Deploy data to the executable folder (only when CMake is run)
# @see http://www.cmake.org/cmake/help/v3.0/command/file.html
file(COPY HandFinder/wristband.txt DESTINATION ${PROJECT_BINARY_DIR})
file(COPY Energy/PoseSpace_PCA DESTINATION ${PROJECT_BINARY_DIR})
file(COPY Calibration/default.calib DESTINATION ${PROJECT_BINARY_DIR})
file(COPY Legacy/fingerfist.mat DESTINATION ${PROJECT_BINARY_DIR})

#--- To simplify our life
include_directories(../) # #include "tracker/..."

#--- Add library to the dependencies
if(WITH_CUDA)
    list(APPEND LIBRARIES cudax)
endif()

#--- Generate moc files (anywhere you have Q_OBJECT)
#set(CMAKE_AUTOMOC OFF)
add_library(htracksdk STATIC ${SOURCES} ${HEADERS} ${SHADERS_QRC})
target_link_libraries(htracksdk ${LIBRARIES})

