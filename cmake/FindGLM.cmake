# FIND_PACKAGE (GLM REQUIRED)
# INCLUDE_DIRECTORIES (${GLM_INCLUDE_DIRS})
# ADD_EXECUTABLE (executable ${YOUR_EXECUTABLE_SRCS})

if(WIN32)
    set(GLM_INCLUDE_DIRS "C:/Developer/include")
else()
FIND_PATH( GLM_INCLUDE_DIRS glm/glm.hpp
    $ENV{GLM_DIR}
    /usr/local/include
    /usr/include
    /opt/include
    ${CMAKE_SOURCE_DIR}/external)
endif()

SET(GLM_FOUND "NO")
IF(GLM_INCLUDE_DIRS)
    SET(GLM_FOUND "YES")
ENDIF()
