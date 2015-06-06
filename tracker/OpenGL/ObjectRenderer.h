#pragma once
#include "util/gl_wrapper.h"
#include <Eigen/Dense>
#include <QGLBuffer>
#include <QGLShaderProgram>
#include <QOpenGLVertexArrayObject>
#include <iostream>

/// Superclass for any renderer
class ObjectRenderer{
protected:
    QGLShaderProgram program;
    QOpenGLVertexArrayObject vao;
    
public:
    GLuint program_id(){ return program.programId(); }
    void set_uniform(const char* name, const Eigen::Matrix4f& value);
    void set_uniform(const char* name, const Eigen::Matrix3f& value);
    void set_uniform(const char* name, float value);
    void set_texture(const char* name, int value /*GL_TEXTURE_?*/);
    void set_uniform_uint(const char* name, unsigned int value);
    
public:
    virtual ~ObjectRenderer(){} ///< safe polymorphic destruct
    virtual void init(){}
    virtual void render()=0;
};

