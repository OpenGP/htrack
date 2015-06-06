#include "KinectDataRenderer.h"
#include "tracker/Data/Camera.h"
#include "util/mylogger.h"

struct Grid{
    std::vector<unsigned int> indices;
    std::vector<GLfloat> vertices;
    std::vector<GLfloat> texcoords;
    Grid(int grid_width, int grid_height){
        ///--- So that we don't have to bother with connectivity data structure!!
        int primitive_restart_idx = 0xffffffff;
        glPrimitiveRestartIndex(primitive_restart_idx);
        glEnable(GL_PRIMITIVE_RESTART);
        
        int skip = 1;
        
        ///--- Vertices
        for (int row = 0; row < grid_height; row+=skip) {
            for (int col = 0; col < grid_width; col+=skip) {
                Scalar x = col;
                Scalar y = row;
                vertices.push_back(x); /// i [0...width]
                vertices.push_back(y); /// y [0...height]
            }
        }
        
        ///--- TexCoords
        for (int row = 0; row < grid_height; row+=skip) {
            for (int col = 0; col < grid_width; col+=skip) {
                Scalar x = col/((Scalar)grid_width);
                Scalar y = row/((Scalar)grid_height);
                texcoords.push_back(x); /// u [0,1]
                texcoords.push_back(y); /// v [0,1]
            }
        }
    
        ///--- Faces
        for (int row = 0; row < grid_height - 1; row+=skip) {
            for (int col = 0; col < grid_width; col+=skip) {
                indices.push_back((row + 1) * grid_width + col);
                indices.push_back(row * grid_width + col);
            }
            indices.push_back(primitive_restart_idx);
        }
    }
};

void KinectDataRenderer::init(Camera *camera)
{
    this->camera = camera;

    ///--- Create vertex array object
    if(!vao.isCreated()){
        bool success = vao.create();
        assert(success);
        vao.bind();
    }
    
    ///--- Load/compile shaders
    if (!program.isLinked()) {
        const char* vshader = ":/KinectDataRenderer/KinectDataRenderer_vshader.glsl";
        const char* fshader = ":/KinectDataRenderer/KinectDataRenderer_fshader.glsl";
        bool vok = program.addShaderFromSourceFile(QGLShader::Vertex, vshader);
        bool fok = program.addShaderFromSourceFile(QGLShader::Fragment, fshader);
        bool lok = program.link ();
        assert(lok && vok && fok);
        bool success = program.bind();
        assert(success); 
    }
    
    ///--- Used to create connectivity
    Grid grid(camera->width(),camera->height());
    
    ///--- Create vertex buffer/attributes "position"
    {
        bool success = vertexbuffer.create();
        assert(success);
        vertexbuffer.setUsagePattern( QGLBuffer::StaticDraw );
        success = vertexbuffer.bind();
        assert(success);
        vertexbuffer.allocate( grid.vertices.data(), grid.vertices.size() * sizeof(GLfloat) );
        program.setAttributeBuffer("vpoint", GL_FLOAT, 0, 2 );
        program.enableAttributeArray("vpoint");
    }
    
    ///--- Create vertex buffer/attributes "uv"
    {
        bool success = uvbuffer.create();
        assert(success);
        uvbuffer.setUsagePattern( QGLBuffer::StaticDraw );
        success = uvbuffer.bind();
        assert(success);
        uvbuffer.allocate( grid.texcoords.data(), grid.texcoords.size() * sizeof(GLfloat) );
        program.setAttributeBuffer("uv", GL_FLOAT, 0, 2 );
        program.enableAttributeArray("uv");
    }
    
    ///--- Create the index "triangle" buffer
    {
        bool success = indexbuffer.create();
        assert(success);
        indexbuffer.setUsagePattern( QGLBuffer::StaticDraw ); 
        success = indexbuffer.bind();
        assert(success);
        indexbuffer.allocate( grid.indices.data(), grid.indices.size() * sizeof(unsigned int) );
    }   
    
    ///--- Create texture to colormap the point cloud
    {
        // const int sz=2; GLfloat tex[3*sz] = {/*green*/ 0.000, 1.000, 0, /*red*/ 1.000, 0.000, 0,};
        // const int sz=2; GLfloat tex[3*sz] = {/*gray*/ .1, .1, .1, /*black*/ 0.8, 0.8, 0.8};
        const int sz=3; GLfloat tex[3*sz] = {/*red*/ 1.000, 0.000, 0, /*yellow*/ 1.0, 1.0, 0.0, /*green*/ 0.000, 1.000, 0};
        glActiveTexture(GL_TEXTURE2);
        glGenTextures(1, &texture_id_cmap);
        glBindTexture(GL_TEXTURE_1D, texture_id_cmap);
        glTexImage1D(GL_TEXTURE_1D, 0, GL_RGB, sz, 0, GL_RGB, GL_FLOAT, tex);
        glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameterf(GL_TEXTURE_1D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameterf(GL_TEXTURE_1D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        program.setUniformValue("colormap", 2 /*GL_TEXTURE2*/);
    }
    
    ///--- @todo upload data to do inverse projection
    set_uniform("inv_proj_matrix",camera->inv_projection_matrix());
    
    ///--- upload near/far planes
    set_zNear(camera->zNear());
    set_zFar(camera->zFar());
    // set_alpha(1.0); ///< default no alpha blending
    set_alpha(.7); ///< default no alpha blending
    set_discard_cosalpha_th(.3);  ///< default sideface clipping
    
    ///--- save for glDrawElements
    num_indexes = grid.indices.size();
    num_vertices = grid.vertices.size();
    
    ///--- Avoid pollution
    program.release();
    vao.release();
}

void KinectDataRenderer::setup(GLuint texture_id_color, GLuint texture_id_depth)
{ 
    // LOG(INFO) << "KinectDataRenderer::setup";
    this->texture_id_color = texture_id_color; 
    this->texture_id_depth = texture_id_depth;

    program.bind();
    glUniform1i(glGetUniformLocation(program.programId(), "tex_color"), 0 /*GL_TEXTURE_0*/);
    glUniform1i(glGetUniformLocation(program.programId(), "tex_depth"), 1 /*GL_TEXTURE_1*/);            
    program.release();
}

void KinectDataRenderer::render()
{ 
    if(texture_id_color==0 || texture_id_depth==0) return;
    
    if(alpha < 1.0){
        glEnable(GL_BLEND);
        glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    } else {
        glDisable(GL_BLEND);
    }
    
    vao.bind();
    program.bind();        
        ///--- color texture
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, texture_id_color);
        ///--- depth texture
        glActiveTexture(GL_TEXTURE1);
        glBindTexture(GL_TEXTURE_2D, texture_id_depth);
        ///--- colormap texture
        glActiveTexture(GL_TEXTURE2);
        glBindTexture(GL_TEXTURE_1D, texture_id_cmap);
       
#define KINECT_DRAW_MESH
#ifdef KINECT_DRAW_MESH
        ///--- draw (as triangles)
        glDrawElements(GL_TRIANGLE_STRIP, num_indexes, GL_UNSIGNED_INT, 0);
#else
#ifdef __APPLE__
    #error you will not see any input data!!
#endif
        ///--- draw (as point array)
        glEnable(GL_PROGRAM_POINT_SIZE);
            glDrawArrays(GL_POINTS, 0, num_vertices );
        glDisable(GL_PROGRAM_POINT_SIZE);
#endif
        ///--- draw (as indexed points)
        // glDrawElements(GL_POINTS, num_indexes, GL_UNSIGNED_INT, 0);
    program.release();
    vao.release();
}

void KinectDataRenderer::set_discard_cosalpha_th(float val){
    set_uniform("discard_cosalpha_th", val);    
}

void KinectDataRenderer::set_alpha(float alpha){
    set_uniform("alpha", alpha);
    this->alpha = alpha;
}
void KinectDataRenderer::set_zNear(float alpha){
    set_uniform("zNear",alpha);    
}
void KinectDataRenderer::set_zFar(float alpha){
    set_uniform("zFar",alpha);    
}
void KinectDataRenderer::enable_colormap(bool enable){
    set_uniform("enable_colormap", (enable)?+1.0f:-1.0f);           
}
