if(NOT WITH_CUDA)
    return()
endif()

#--- Dummy rule to show whole cudax folder in explorer
file(GLOB CUDA_FILES "*")
add_custom_target(cudax_files SOURCES ${CUDA_FILES})

#--- C++ sources (non-kernel) that need cuda are appended
file(GLOB_RECURSE CUDAX_CPP "${CMAKE_CURRENT_LIST_DIR}/*.cpp")
file(GLOB_RECURSE CUDAX_CU  "${CMAKE_CURRENT_LIST_DIR}/*.cu")
file(GLOB_RECURSE CUDAX_H   "${CMAKE_CURRENT_LIST_DIR}/*.h")

#--- Create the cudax static library & add it
INCLUDE_DIRECTORIES(.)
INCLUDE_DIRECTORIES(../) #so we can say #include "cudax/..."
cuda_add_library(cudax STATIC ${CUDAX_CPP} ${CUDAX_CU} ${CUDAX_H})

