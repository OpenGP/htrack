#pragma once
#include "MLogger.h"

#ifdef __clang__
    #pragma clang diagnostic ignored "-Wunused-function"
#endif

#ifndef WITH_LOGGER
    /// Back-compatibility for easylogging++
    #define INFO  "[INFO]"
    #define FATAL "[FATAL]"
    #define LOG(x) mDebug()
    #define CHECK_BOUNDS(x,y,z) ((void)0)
    #define LOG_UNREACHABLE LOG(FATAL) ((void)0)
#endif
