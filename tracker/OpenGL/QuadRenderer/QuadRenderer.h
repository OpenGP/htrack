#include "tracker/ForwardDeclarations.h"
#include "tracker/OpenGL/ObjectRenderer.h"

class QuadRenderer : public ObjectRenderer{
public:
    enum RenderMode{Color,Depth};
private:    
    QGLBuffer vertexbuffer = QGLBuffer(QGLBuffer::VertexBuffer);
    GLuint texture_id = 0;
    GLuint colormap_texture_id = 0;
    RenderMode mode = Color;
    Camera* camera;
public:    
    QuadRenderer(RenderMode mode, Camera* camera):mode(mode), camera(camera){}
    void init();
    void render();
    void setup(GLuint texid);
};
