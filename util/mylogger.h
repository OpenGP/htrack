#pragma once
#include <fstream> 

#ifdef __clang__
    #pragma clang diagnostic ignored "-Wunused-function"
#endif

#ifdef WITH_LOGGER
    #include "easylogging++.h"
#else
    #define INFO "[INFO]"
    #define FATAL ((void)0)
    #include <QDebug>
    #include "qDebug_helpers.h"
    inline void _foo(){}
    #define TIMED_BLOCK(x,y) _foo();
    #define TIMED_SCOPE(x,y) _foo();
    #define TIMED_FUNC(x) _foo();
    #define LOG(x) qDebug()
    #define CHECK_BOUNDS(x,y,z) ((void)0)
    #define LOG_UNREACHABLE LOG(FATAL) ((void)0)
    #define CHECK(val) if(true) std::cout
    #define CHECK_NOTNULL(val) if(val==NULL){ std::cout << "!!!CHECK_NOTNULL: " << __FILE__ << " " << __LINE__ << std::endl; exit(0); }
#endif

static void setup_logger(){
#ifdef WITH_LOGGER
#endif
}
