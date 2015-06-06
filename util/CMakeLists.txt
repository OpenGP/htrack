
#--- Dummy target to see headers
include_directories(.)
file(GLOB_RECURSE UTIL_HEADERS "*.h")
add_custom_target(util_headers SOURCES ${UTIL_HEADERS})
