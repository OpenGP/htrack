set(WITH_CUDA TRUE)
add_definitions(-DWITH_CUDA)

# Anastasia: what is this for?
if(WIN32)
   # set(CUDA_TOOLKIT_ROOT_DIR "C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA/v6.5")
   set(CUDA_TOOLKIT_ROOT_DIR "C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA/v7.0")
endif()

#--- Import CUDA/CUBLAS
find_package(CUDA REQUIRED)
include_directories(${CUDA_INCLUDE_DIRS})
list(APPEND LIBRARIES ${CUDA_LIBRARIES})
list(APPEND LIBRARIES ${CUDA_CUBLAS_LIBRARIES})

#--- For matrix operations within Kernels (Eigen not supported)
find_package(GLM REQUIRED)
include_directories(${GLM_INCLUDE_DIRS})
add_definitions(-DGLM_FORCE_CUDA) #< as mentioned in docs

#--- Card needs appropriate version
site_name(HOSTNAME)
message(STATUS "HOSTNAME ${HOSTNAME}")

if(HOSTNAME STREQUAL "lagrange") #--- Andrea/Linux
    set(CUDA_NVCC_FLAGS "-gencode arch=compute_50,code=sm_50")# GTX980
    set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -w -Xcompiler -fPIC" )
elseif(HOSTNAME STREQUAL "IC-LGG-DPC-06") #--- Andrea,Fridge,Windows
    set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -gencode arch=compute_50,code=sm_50") # GTX980
elseif(HOSTNAME STREQUAL "tsf-460-wpa-4-107.epfl.ch") # Euler MAC-PRO Retina
    set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -gencode arch=compute_30,code=sm_30") # Geforce 650m
elseif(HOSTNAME STREQUAL "waluigi") #--- Matthias/Linux
    set(CUDA_NVCC_FLAGS "-gencode arch=compute_30,code=sm_30")# GTX660 Ti
    set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -w -Xcompiler -fPIC" )
elseif(HOSTNAME STREQUAL "nastylinux") #--- Anastasia
    set(CUDA_NVCC_FLAGS "-gencode arch=compute_30,code=sm_30")# GTX660 Ti
    set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -w -Xcompiler -fPIC -D__CUDACC__" )
elseif(HOSTNAME STREQUAL "thp") #--- THP/Alienware
    set(CUDA_NVCC_FLAGS "-gencode arch=compute_30,code=sm_30")# GTX780M

#--- Generic
else()
    #set(CUDA_NVCC_FLAGS "-gencode arch=compute_32,code=sm_32") # invalid device function
    set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -gencode arch=compute_30,code=sm_30") # Geforce 650m (MBPro Retina)
    set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -gencode arch=compute_32,code=sm_32") # ?
    set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -gencode arch=compute_35,code=sm_35") # ?
    set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -gencode arch=compute_50,code=sm_50") # GTX980 (Win/Linux Machine)
endif()

#--- Enable debugging flags
#set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -g") # HOST debug mode
#set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -G") # DEV debug mode
#set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -lineinfo")
set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -DNDEBUG") #< disable asserts

if(WIN32)
    set(CUDA_PROPAGATE_HOST_FLAGS True)
    if (CMAKE_BUILD_TYPE STREQUAL "Release")
        set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -O3")
        set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -use_fast_math")
    endif()
else()
    #--- CUDA doesn't like "--compiler-options -std=c++11"
    set(CUDA_PROPAGATE_HOST_FLAGS False)
    set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -std c++11")
    set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -O3")
    set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -use_fast_math")
endif()

message(STATUS "CUDA_PROPAGATE_HOST_FLAGS: ${CUDA_PROPAGATE_HOST_FLAGS}")
message(STATUS "CUDA_HOST_COMPILER: ${CUDA_HOST_COMPILER}")
message(STATUS "CUDA_NVCC_FLAGS: ${CUDA_NVCC_FLAGS}")
