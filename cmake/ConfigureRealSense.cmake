# TODO: use the find_file macro

if(WIN32)
    set(REALSENSE_INCLUDE_DIR "C:/Developer/RealSense/RSSDK/include")
    set(REALSENSE_UTILITY_DIR "C:/Developer/RealSense/RSSDK/sample/common/include")

    if(${CMAKE_BUILD_TYPE} STREQUAL "Debug")
        set(REALSENSE_LIBRARY "C:/Developer/RealSense/RSSDK/lib/x64/libpxcmd_d.lib")
            set(REALSENSE_UTILITY_LIBRARY "C:/Developer/RealSense/RSSDK/sample/common/lib/x64/v120/libpxcutilsmd_d.lib")
    endif()

    if(${CMAKE_BUILD_TYPE} STREQUAL "Release")
        set(REALSENSE_LIBRARY "C:/Developer/RealSense/RSSDK/lib/x64/libpxcmd.lib")
            set(REALSENSE_UTILITY_LIBRARY "C:/Developer/RealSense/RSSDK/sample/common/lib/x64/v120/libpxcutilsmd.lib")
    endif()


    include_directories(${REALSENSE_INCLUDE_DIR} ${REALSENSE_UTILITY_DIR})

    list(APPEND LIBRARIES ${REALSENSE_LIBRARY})
    list(APPEND LIBRARIES ${REALSENSE_UTILITY_LIBRARY})

    #--- DEBUG
    # message(STATUS "REALSENSE_LIBRARY ${REALSENSE_LIBRARY}")
    # message(STATUS "REALSENSE_UTILITY_LIBRARY ${REALSENSE_UTILITY_LIBRARY}")

    #--- Mark as available
    add_definitions(-DHAS_REALSENSE)
endif()
