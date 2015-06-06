#pragma once
#include "tracker/Types.h"
#include "tracker/OpenGL/ObjectRenderer.h"

class Segment_renderer : public ObjectRenderer{
    int num_segments = 0;
    QGLBuffer vertexbuffer = QGLBuffer(QGLBuffer::VertexBuffer);
    QGLBuffer vsize_buf = QGLBuffer(QGLBuffer::VertexBuffer);
    QGLBuffer vcolor_buf = QGLBuffer(QGLBuffer::VertexBuffer);

public:
    void setup(Matrix_3xN* points, Matrix_3xN* colors = NULL);
    void init();
    void render();
    void perturb();
};
