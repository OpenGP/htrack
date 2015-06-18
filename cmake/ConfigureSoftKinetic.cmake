#--- OpenNI2
find_package(SoftKinetic)  #NOT REQUIRED (mac doesn't have it)
if(SOFTKINETIC_FOUND)
    include_directories(${SOFTKINETIC_INCLUDE_DIR})
    list(APPEND LIBRARIES ${SOFTKINETIC_LIBRARIES})
    add_definitions(-DHAS_SOFTKINETIC)
endif()
