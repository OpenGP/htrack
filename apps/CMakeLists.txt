cmake_minimum_required(VERSION 2.8)
project(htrack-helloworlds)

#--- Gets rid of annoying CMake 3 warnings
if(NOT (CMAKE_MAJOR_VERSION LESS 3))
    cmake_policy(SET CMP0043 OLD)
    cmake_policy(SET CMP0020 OLD)
    cmake_policy(SET CMP0005 OLD)
endif()

#--- Helloworld applications
add_subdirectory(helloworld)
add_subdirectory(helloworld_opencv)
add_subdirectory(helloworld_qtopengl)
add_subdirectory(helloworld_atb) # only for htrack-atb
add_subdirectory(helloworld_cudagl)
add_subdirectory(helloworld_cublas)
add_subdirectory(helloworld_thrust)
add_subdirectory(helloworld_openmp) # linux-only
add_subdirectory(helloworld_qglviewer) # only for htrack-qt

#--- Sensors Testing
add_subdirectory(helloworld_realsense) # windows only!!
add_subdirectory(helloworld_openni) # any os
