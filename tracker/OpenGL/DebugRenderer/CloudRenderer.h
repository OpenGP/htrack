#pragma once
#include "tracker/Types.h"
#include "tracker/OpenGL/ObjectRenderer.h"

class CloudRenderer : public ObjectRenderer{
    int num_points = 0;
    QGLBuffer vertexbuffer = QGLBuffer(QGLBuffer::VertexBuffer);
    QGLBuffer vsize_buf = QGLBuffer(QGLBuffer::VertexBuffer);
    QGLBuffer vcolor_buf = QGLBuffer(QGLBuffer::VertexBuffer);

public:
    void setup(Matrix_3xN* points, Matrix_3xN* colors = NULL, VectorN* sizes = NULL);
    void init();
    void render();
    // void perturb();
};
