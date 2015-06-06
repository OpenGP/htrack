#pragma once

#include <stdio.h>
#include <stdarg.h>
// #include <qglobal.h>

#ifndef NDEBUG
    #define NDEBUG
#endif

#ifdef NDEBUG
#ifndef assertf
#define assertf(x,fmt,...)\
if (!(x)){\
    char buf[2048];\
    char* cbuf=&(buf[0]);\
    cbuf += snprintf(cbuf,2048,"%s","[ASSERT] '");\
    cbuf += snprintf(cbuf,2048,fmt,##__VA_ARGS__);\
    cbuf += snprintf(cbuf,2048,"' at file '%s' line '%d'",__FILE__,__LINE__);\
    cbuf += snprintf(cbuf,2048,"\n");\
    perror(buf);\
    abort();\
}
#endif
#else
# define assertf(x,fmt,...) ((void)0) /* ASSERTF does nothing when not debugging */
#endif

//fprintf(stderr, buf);
