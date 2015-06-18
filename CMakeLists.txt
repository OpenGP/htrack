cmake_minimum_required(VERSION 2.8)
project(htrack)

#--- Gets rid of annoying CMake 3 warnings
if(NOT (CMAKE_MAJOR_VERSION LESS 3))
    cmake_policy(SET CMP0043 OLD)
    cmake_policy(SET CMP0020 OLD)
    cmake_policy(SET CMP0005 OLD)
endif()

#--- Where to find the FindPackageName.cmake
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)

#--- Where exes/libs will be deployed
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR})
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${PROJECT_BINARY_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_DEBUG ${PROJECT_BINARY_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${PROJECT_BINARY_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE ${PROJECT_BINARY_DIR})

#--- Compiler setup
include(cmake/ConfigureCompiler.cmake)

#--- Configure the libraries
include(cmake/ConfigureQt.cmake)
include(cmake/ConfigureOpenGL.cmake)
include(cmake/ConfigureGLEW.cmake)
include(cmake/ConfigureEigen.cmake)
include(cmake/ConfigureOpenCV.cmake)
include(cmake/ConfigureOpenNI.cmake)
include(cmake/ConfigureRealSense.cmake)
include(cmake/ConfigureSoftKinetic.cmake)
include(cmake/ConfigureCUDA.cmake)
include(cmake/ConfigureAntTweakBar.cmake)

#--- sub-projects
add_subdirectory(util)
add_subdirectory(cudax)
add_subdirectory(tracker)

#--- Choose which sensor to use
include(cmake/ConfigureSensor.cmake)
add_definitions(${DEFINITION_SENSOR})

add_subdirectory(apps/htrack_atb)
add_subdirectory(apps/htrack_qt)
#add_subdirectory(apps/synthgen) #<<< NOT READY
#add_subdirectory(apps/colopick) #<<< NOT READY
