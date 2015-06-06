#include "Segment_renderer.h"

#include "tracker/Types.h"
#include "util/mylogger.h"

void Segment_renderer::setup(Matrix_3xN* segments, Matrix_3xN* colors) {
    CHECK_NOTNULL(segments);
    if(colors) CHECK(segments->cols() == colors->cols());
    this->num_segments = segments->cols();
        
    vao.bind();    
    program.bind();    
    
    ///--- Create vertex buffer/attributes "position"
    {
        bool success = vertexbuffer.create();
        assert(success);
        vertexbuffer.setUsagePattern( QGLBuffer::StaticDraw );
        success = vertexbuffer.bind();
        assert(success);
        vertexbuffer.allocate( segments->data(), sizeof(Scalar) * segments->size() );
        program.setAttributeBuffer("vpoint", GL_FLOAT, 0, 3 );
        program.enableAttributeArray("vpoint");
    }

    ///--- Create vertex buffer/attributes "colors"
    if(!colors){
        static Matrix_3xN _colors(segments->rows(), segments->cols());
        _colors.row(0).array().setConstant(1); ///< force red
        colors = &_colors;
    }
    {
        bool success = vcolor_buf.create();
        assert(success);
        vcolor_buf.setUsagePattern( QGLBuffer::StaticDraw );
        success = vcolor_buf.bind();
        assert(success);
        vcolor_buf.allocate( colors->data(), sizeof(Scalar) * segments->size());
        program.setAttributeBuffer("vcolor", GL_FLOAT, 0, 3 );
        program.enableAttributeArray("vcolor");        
    }
    
    ///--- Create vertex buffer/attributes "sizes"
    {
        static VectorN _sizes(segments->cols());
        VectorN* sizes = &_sizes;
        bool success = vsize_buf.create(); assert(success);
        vsize_buf.setUsagePattern( QGLBuffer::StaticDraw );
        success = vsize_buf.bind(); assert(success);
        vsize_buf.allocate( colors->data(), sizeof(Scalar) * sizes->size() );
        program.setAttributeBuffer("vsize", GL_FLOAT, 0, 1 );
        program.enableAttributeArray("vsize");     
    }
    
    program.release();
    vao.release();
}

void Segment_renderer::init() {
    ///--- Load/compile shaders
    if (!program.isLinked()) {
        const char* vshader = ":/CloudRenderer_vshader.glsl";
        const char* fshader = ":/CloudRenderer_fshader.glsl";
        bool vok = program.addShaderFromSourceFile(QGLShader::Vertex, vshader);
        bool fok = program.addShaderFromSourceFile(QGLShader::Fragment, fshader);
        bool lok = program.link ();
        assert(lok && vok && fok);
        bool success = program.bind();
        assert(success); 
        // printf("Compiled shader #%d from '%s' '%s'\n", program.programId(), vshader, fshader);
    }

    ///--- Create vertex array object
    {
        bool success = vao.create();
        assert(success);
    }
    
    ///--- Avoid pollution
    program.release();
    vao.release();
}

void Segment_renderer::render() {
    if (num_segments==0) return;
    vao.bind();
    program.bind();
    glEnable(GL_PROGRAM_POINT_SIZE);
    glDrawArrays(GL_LINES, 0, num_segments);
    glDisable(GL_PROGRAM_POINT_SIZE);
    program.release();
    vao.release();
}
