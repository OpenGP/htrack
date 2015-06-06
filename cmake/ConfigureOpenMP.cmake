if(UNIX)
     # only for g++ or for the Clang+OpenMP compiler
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
elseif(WIN32)
    message(STATUS "OpenMP not tested on windows!")
endif()

add_definitions(-DWITH_OPENMP)
