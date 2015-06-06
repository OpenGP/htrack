#include "CloudRenderer.h"

#include "tracker/Types.h"
#include "util/mylogger.h"

void CloudRenderer::setup(Matrix_3xN* points, Matrix_3xN* colors, VectorN* sizes) {
    if (!program.isLinked()) {
        init(); // setup is called before init, when shader is not yet compiled...
    }

    CHECK_NOTNULL(points);
    if(colors) CHECK(points->cols() == colors->cols());
    if(sizes) CHECK(sizes->size() == points->cols());
    this->num_points = points->cols();    
        
    vao.bind();    
    program.bind();
    
// #define CLOUD_PRINT_DATA
#ifdef CLOUD_PRINT_DATA    
    std::cout << "data" << std::endl;
    for(int i=0; i<points->cols(); i++)
        std::cout << points->col(i).transpose() << std::endl;
#endif
    
    ///--- Create vertex buffer/attributes "position"
    {
        bool success = vertexbuffer.create();
        assert(success);
        vertexbuffer.setUsagePattern( QGLBuffer::StaticDraw );
        success = vertexbuffer.bind();
        assert(success);
        vertexbuffer.allocate( points->data(), sizeof(Scalar) * points->size() );
        program.setAttributeBuffer("vpoint", GL_FLOAT, 0, 3 );
        program.enableAttributeArray("vpoint");
    }

    ///--- Create vertex buffer/attributes "colors"
    if(!colors){
        static Matrix_3xN _colors(points->rows(), points->cols());
        _colors.row(0).array().setConstant(1); ///< force red
        colors = &_colors;
    }
    {
        bool success = vcolor_buf.create();
        assert(success);
        vcolor_buf.setUsagePattern( QGLBuffer::StaticDraw );
        success = vcolor_buf.bind();
        assert(success);
        vcolor_buf.allocate( colors->data(), sizeof(Scalar) * points->size() );
        program.setAttributeBuffer("vcolor", GL_FLOAT, 0, 3 );
        program.enableAttributeArray("vcolor");        
    }
    
    ///--- Create vertex buffer/attributes "sizes"
    if(!sizes){
        static VectorN _sizes(points->cols());
        sizes = &_sizes;
    }
    {
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

void CloudRenderer::init() {
    ///--- Load/compile shaders
    if (!program.isLinked()) {
        const char* vshader = ":/CloudRenderer/CloudRenderer_vshader.glsl";
        const char* fshader = ":/CloudRenderer/CloudRenderer_fshader.glsl";
        bool vok = program.addShaderFromSourceFile(QGLShader::Vertex, vshader);
        bool fok = program.addShaderFromSourceFile(QGLShader::Fragment, fshader);
        bool lok = program.link ();
        assert(lok && vok && fok);
        bool success = program.bind();
        assert(success); 
        // printf("Compiled shader #%d from '%s' '%s'\n", program.programId(), vshader, fshader);
    }

    ///--- Create vertex array object
    if(!vao.isCreated()){
        bool success = vao.create();
        assert(success);
    }
    
    ///--- Avoid pollution
    program.release();
    vao.release();
}

void CloudRenderer::render() {
    if (num_points==0) return;
    vao.bind();
    program.bind();
    glEnable(GL_PROGRAM_POINT_SIZE);
    glDrawArrays(GL_POINTS, 0, num_points);
    glDisable(GL_PROGRAM_POINT_SIZE);
    program.release();
    vao.release();
}

#ifdef TEST_PERTURB_CLOUD
void CloudRenderer::perturb(){
    std::cout << "perturb/2!!!" << std::endl;
    vao.bind();
        vertexbuffer.bind();
        int num_bytes = 3*sizeof(float)*cloud->size();
        vector<float> data(3*cloud->size());
        // vertexbuffer.read(0, data.data(), num_bytes);
        for(int i=0; i<data.size(); i++)
            data[i] += Matlab::randn(0,5);
        vertexbuffer.write(0, data.data(), num_bytes);
        vertexbuffer.release();
    vao.release();
}
#endif
