# @see http://doc.qt.io/qt-5/cmake-manual.html
#--- Tell cmake where to find Qt5. (Unfortunately cmake3 does not support Qt5 yet)

#--- Debug
#message(STATUS QT5_CMAKE_DIR $ENV{QT5_CMAKE_DIR})
#message(STATUS CMAKE_PREFIX_PATH $ENV{CMAKE_PREFIX_PATH})

if(DEFINED ENV{CMAKE_PREFIX_PATH})
    # If user has specified it, just use it...
    set(CMAKE_PREFIX_PATH $ENV{CMAKE_PREFIX_PATH})
    message(STATUS "Qt5: Using environment CMAKE_PREFIX_PATH: ${CMAKE_PREFIX_PATH}")

elseif(DEFINED ENV{QT5_CMAKE_DIR})
    # User could have also specified QT5_CMAKE_DIR
    set(CMAKE_PREFIX_PATH $ENV{QT5_CMAKE_DIR})
    message(STATUS "Qt5: Using environment variable QT5_CMAKE_DIR: ${CMAKE_PREFIX_PATH}")

else()
    # Otherwise just resort to some built-in stuff
    if(WIN32)
        set(CMAKE_PREFIX_PATH "C:/Developer/Qt/5.4/msvc2013_64_opengl/lib/cmake")
    elseif(APPLE)
        set(CMAKE_PREFIX_PATH "/usr/local/Cellar/qt5/5.4.1/lib/cmake")
    elseif(UNIX)
        set(CMAKE_PREFIX_PATH "/usr/local/qt/5.3/gcc_64/lib/cmake")
    endif()
    message(STATUS "!!!WARNING Qt5: resorting to default Qt5 paths (might fail): ${CMAKE_PREFIX_PATH}")
endif()

#--- Needed for GUI
find_package(Qt5Widgets REQUIRED)
include_directories(${Qt5Widgets_INCLUDE_DIRS})
list(APPEND LIBRARIES Qt5::Widgets)

#--- Needed for OpenGL
find_package(Qt5OpenGL REQUIRED)
include_directories(${Qt5OpenGL_INCLUDE_DIRS})
list(APPEND LIBRARIES Qt5::OpenGL)

#--- Needed for the QGLViewer library
find_package(Qt5Xml REQUIRED)
include_directories(${Qt5Xml_INCLUDE_DIRS})
list(APPEND LIBRARIES Qt5::Xml)

