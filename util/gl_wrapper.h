#pragma once

#ifdef _WIN32
    /// fix windows.h problems
    #define NOMINMAX
    /// fix no storage class or type specifier gl.h
    #include <windows.h>
#endif

///--- GLEW
//#ifdef WITH_GLEW
#include <GL/glew.h> ///< must be before anything else gl'ish
#include <iostream>
inline void initialize_glew(){
    glewExperimental = true;
    if (glewInit() != GLEW_NO_ERROR){

        fprintf(stderr, "Failed to initialize GLEW\n");
        exit(EXIT_FAILURE);
    }
}

///--- Linux needs extensions for framebuffers
#if __unix__
#define GL_GLEXT_PROTOTYPES 1
#include <GL/gl.h>
#include <GL/glext.h>
#endif

#if _WIN32
#include <GL/gl.h>
#endif

#ifdef __APPLE__
#include "OpenGL/glu.h"
#else
#include "GL/glu.h"
#endif

#include "check_error_gl.h"
