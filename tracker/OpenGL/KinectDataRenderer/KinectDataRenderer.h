#pragma once
#include "tracker/ForwardDeclarations.h"
#include "tracker/Types.h"
#include "tracker/OpenGL/ObjectRenderer.h"

class KinectDataRenderer : public ObjectRenderer{
    QGLBuffer vertexbuffer = QGLBuffer(QGLBuffer::VertexBuffer);
    QGLBuffer uvbuffer = QGLBuffer(QGLBuffer::VertexBuffer);
    QGLBuffer indexbuffer = QGLBuffer(QGLBuffer::IndexBuffer);
    int num_indexes = 0;
    int num_vertices = 0;
    GLuint texture_id_color = 0;
    GLuint texture_id_depth = 0;
    GLuint texture_id_cmap = 0;
    Camera* camera = NULL;
    float alpha = 1.0;
public:
    void init(Camera* camera);
    void setup(GLuint texture_id_color, GLuint texture_id_depth);
    void render();
    
public:
    void set_alpha(float alpha);
    void set_zNear(float alpha);
    void set_zFar(float alpha);
    void enable_colormap(bool enable);
    void set_discard_cosalpha_th(float val);
};
