cmake_minimum_required(VERSION 2.8)
get_filename_component(PROJECT ${CMAKE_CURRENT_LIST_DIR} NAME)
project(${PROJECT})

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/../../cmake)
include(../../cmake/ConfigureCompiler.cmake)
include(../../cmake/ConfigureQt.cmake)
include(../../cmake/ConfigureCUDA.cmake)
include(../../cmake/ConfigureEigen.cmake)
include_directories(../../)

#--- only compile a tiny fraction of cudax
set(CUDAX_CU "../../cudax/outer_product.cu")
cuda_add_library(cudax STATIC ${CUDAX_CU})
list(APPEND LIBRARIES cudax)

#--- exec1
add_executable(${PROJECT}_simple main_simple.cpp)
target_link_libraries(${PROJECT}_simple ${LIBRARIES})

#--- exec2 (needs gnuplot!)
if(UNIX AND NOT APPLE)
    add_executable(${PROJECT}_perfomance  main_performance.cpp)
    target_link_libraries(${PROJECT}_perfomance ${LIBRARIES})
endif()

