#include "QuadRenderer.h"

#include "util/gl_wrapper.h"
#include "util/mylogger.h"
#include "tracker/Data/Camera.h"

namespace quad{
const unsigned int n_vertices  = 4;
static float vpoint[] = {
    0.0000f,0.0000f,-.99000f,
    1.0000f,0.0000f,-.99000f,
    0.0000f,1.0000f,-.99000f,
    1.0000f,1.0000f,-.99000f};
} // quad::

void QuadRenderer::init()
{
    ///--- Create vertex array object
    if(!vao.isCreated()){
        bool success = vao.create();
        assert(success);
    }
    
    CHECK_ERROR_GL();
    
    if (!program.isLinked()) {
        const char* vshader = ":/QuadRenderer/QuadRenderer_vshader.glsl";
        const char* fshader = (mode==Color)?
            ":/QuadRenderer/QuadRenderer_color_fshader.glsl":
            ":/QuadRenderer/QuadRenderer_depth_fshader.glsl";
        bool vok = program.addShaderFromSourceFile(QGLShader::Vertex, vshader);
        bool fok = program.addShaderFromSourceFile(QGLShader::Fragment, fshader);
        bool lok = program.link ();
        assert(lok && vok && fok);
    }
        
    CHECK_ERROR_GL();

    vao.bind();
    program.bind();
        CHECK_ERROR_GL();
    
        ///--- Create vertex buffer/attributes "position"
        {
            bool success = vertexbuffer.create();
            assert(success);
            vertexbuffer.setUsagePattern( QGLBuffer::StaticDraw );
            success = vertexbuffer.bind();
            assert(success);
            vertexbuffer.allocate( quad::vpoint, 3 * sizeof(float) * quad::n_vertices );
            program.setAttributeBuffer("vpoint", GL_FLOAT, 0, 3 );
            program.enableAttributeArray("vpoint");
        }
        CHECK_ERROR_GL();
    
        ///--- Create texture to colormap depth channel
        if(mode==Depth)
        {
            GLfloat tex[3*2] = {0.000, 1.000, 0, 
                                1.000, 0.000, 0,};
            glActiveTexture(GL_TEXTURE1);
            glGenTextures(1, &colormap_texture_id);
            glBindTexture(GL_TEXTURE_1D, colormap_texture_id);
            glTexImage1D(GL_TEXTURE_1D, 0, GL_RGB, 2, 0, GL_RGB, GL_FLOAT, tex);
            glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameterf(GL_TEXTURE_1D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
            glTexParameterf(GL_TEXTURE_1D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
            program.setUniformValue("colormap", 1 /*GL_TEXTURE_1*/);
        }
        CHECK_ERROR_GL();
        
        ///--- Near/Far planes for depth data
        set_uniform_uint("znear", camera->zNear());
        set_uniform_uint("zfar",  camera->zFar());

    ///--- Avoid pollution
    program.release();
    vao.release();

    CHECK_ERROR_GL();
}

void QuadRenderer::setup(GLuint texture_id)
{ 
    this->texture_id = texture_id; 
    program.bind(); ///< needed? 
        glUniform1i(glGetUniformLocation(program.programId(), "tex"), 0 /*GL_TEXTURE_0*/);
    program.release(); ///< needed?
}

void QuadRenderer::render()
{
    if(texture_id==0) return;
    
    glDisable(GL_DEPTH_TEST);
    vao.bind();
    program.bind();
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, texture_id);
        CHECK_ERROR_GL();
        if(mode==Depth && colormap_texture_id!=0)
        {
            glActiveTexture(GL_TEXTURE1);
            glBindTexture(GL_TEXTURE_1D, colormap_texture_id);
        }
        CHECK_ERROR_GL();
        glDrawArrays(GL_TRIANGLE_STRIP, 0, quad::n_vertices);
    program.release();
    vao.release();
    glEnable(GL_DEPTH_TEST);
    CHECK_ERROR_GL();
}
