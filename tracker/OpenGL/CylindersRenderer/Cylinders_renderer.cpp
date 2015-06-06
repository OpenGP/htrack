#include "Cylinders_renderer.h"

#include "util/gl_wrapper.h"
#include "util/mylogger.h"
#include "tracker/ForwardDeclarations.h"
#include "tracker/Types.h"
#include "tracker/Legacy/geometry/Cylinders.h"
#include "icopill.h"

class Cylindroid{
    QOpenGLVertexArrayObject vao;
    QGLBuffer vertexbuffer = QGLBuffer(QGLBuffer::VertexBuffer);
    QGLBuffer normalbuffer = QGLBuffer(QGLBuffer::VertexBuffer);
    QGLBuffer indexbuffer = QGLBuffer(QGLBuffer::IndexBuffer);
public:
    Cylindroid(QGLShaderProgram& program, 
               float r1_x, float r1_y,
               float r2_x, float r2_y,
               float l)
    {
        assert(program.isLinked());
        program.bind();
                
        ///--- Create an array object to store properties
        {
            bool success = vao.create();
            assert(success);
            vao.bind();
        } 
        
        ///--- Transform  
        Matrix_MxN vertices;
        {
            const Eigen::Map<Matrix_MxN> vertices_orig(icopill::vpoint, 3, icopill::n_vpoint);
            vertices = vertices_orig;            

            size_t n = icopill::n_vpoint;
            size_t h = icopill::n_vpoint/2;

            // collapse center segment
            vertices.block(1,h,1,h).array() -= 1.0f;

            // set bottom and top segment radius
            vertices.block(0,0,3,h).array() *= r1_x;
            vertices.block(0,h,3,h).array() *= r2_x;

            // apply radi
            vertices.block(1,0,2,h).array() *= (r1_y/r1_x);
            vertices.block(1,h,2,h).array() *= (r2_y/r2_x);
                        
            // apply radius ratio and segment length
            vertices.block(1,h,1,h).array() += l;

            /// TODO: also transform normals in there!
        }
        
        ///--- Create vertex buffer/attributes "position"
        {
            bool success = vertexbuffer.create();
            assert(success);
            vertexbuffer.setUsagePattern( QGLBuffer::StaticDraw ); 
            success = vertexbuffer.bind();
            assert(success);
            vertexbuffer.allocate( vertices.data(), 3*sizeof(float)*icopill::n_vpoint );
            program.setAttributeBuffer("vpoint", GL_FLOAT, 0, 3 );
            program.enableAttributeArray("vpoint");
        }
        
        ///--- Create vertex buffer/attributes "normal"
        {
            bool success = normalbuffer.create();
            assert(success);
            normalbuffer.setUsagePattern( QGLBuffer::StaticDraw ); 
            success = normalbuffer.bind();
            assert(success);
            normalbuffer.allocate( icopill::vnormal, 3*sizeof(float)*icopill::n_vnormal );
            program.setAttributeBuffer("vnormal", GL_FLOAT, 0, 3 );
            program.enableAttributeArray("vnormal");
        }
        
        ///--- Create the index "triangle" buffer
        {
            bool success = indexbuffer.create();
            assert(success);
            indexbuffer.setUsagePattern( QGLBuffer::StaticDraw ); 
            success = indexbuffer.bind();
            assert(success);
            indexbuffer.allocate(icopill::findex, sizeof(unsigned int)*icopill::n_findex );
        }
        
        vao.release();
        program.release();
    }

    void render(QGLShaderProgram& program, const Matrix4 &m, size_t id, Vector3& color){ 
        // std::cout << "RENDERING" << std::endl;
        vao.bind(); 

            GLint m_id = glGetUniformLocation(program.programId(), "joint_transform");
            glUniformMatrix4fv(m_id, 1, GL_FALSE, m.data());

            GLint c_id = glGetUniformLocation(program.programId(), "instance_id");
            glUniform1f(c_id, id);
            
            GLint color_id = glGetUniformLocation(program.programId(), "color");
            glUniform3fv(color_id, 1, color.data());

            glDrawElements(GL_TRIANGLES, icopill::n_findex, GL_UNSIGNED_INT, 0 /*why is this zero?*/);

        vao.release();
    }
};

Cylinders_renderer::Cylinders_renderer(Cylinders* c) { this->setup(c); }

Cylinders_renderer::Cylinders_renderer(Cylinders* c, Cylinders_renderer::SHADERMODE mode, const Eigen::Matrix4f& view_projection){
    this->setup(c);
    this->init(mode);
    this->set_uniform("view_projection", view_projection);
}

void Cylinders_renderer::setup(Cylinders *c){
    cylinders = c;
    if(!cylinders->getSkeleton()->hasIDs()){
        cylinders->getSkeleton()->generateIDs();
    }
}

void Cylinders_renderer::init(SHADERMODE mode)
{
    this->mode = mode;
    
    ///--- Load/compile shaders
    {
        QString vshader, fshader;
        if(mode==NORMAL || mode==SKINNY){
            vshader = ":/CylindersRenderer/Cylinders_vshader.glsl";
            fshader = ":/CylindersRenderer/Cylinders_fshader.glsl";
        } else {
            vshader = ":/CylindersRenderer/Cylinders_FB_vshader.glsl";
            fshader = ":/CylindersRenderer/Cylinders_FB_fshader.glsl";
        }
        
        // std::cout << "compiling shaders" << std::endl;
        bool vok = program.addShaderFromSourceFile(QGLShader::Vertex, vshader);
        bool fok = program.addShaderFromSourceFile(QGLShader::Fragment, fshader);
        bool lok = program.link();
        if(!(lok && vok && fok)){
            std::cout << "!!! Shader compile error: " << std::endl;
            std::cout << "vshader: " << vshader.toStdString() << std::endl;
            std::cout << "fshader: " << fshader.toStdString() << std::endl;
            std::cout << "shaders log: " << program.log().toStdString() << std::endl;
            exit(EXIT_FAILURE);
        }
        bool success = program.bind();
        assert(success);
    }
    
    ///--- Instantiate the vertex arrays of the sub-objects
    init_geometry();
}

void Cylinders_renderer::init_geometry(){
    // LOG(INFO) << "Cylinders_renderer::init_geometry()";

    CHECK(cylinders!=NULL);
    cylindroids.clear(); ///< delete previous state
    const std::vector<Segment> &segments = cylinders->getSegments();
    for(size_t i = 0; i < segments.size(); ++i){
        const Segment &s = segments[i];
        if(mode==SKINNY){
            cylindroids.push_back(std::make_shared<Cylindroid>(program, segments_size, segments_size, segments_size, segments_size, s.length));
        } else {
#if 1
            cylindroids.push_back(std::make_shared<Cylindroid>(program, s.radius1, s.radius1_y, s.radius2, s.radius2_y, s.length));
#else
            if(!s.joint->getName().compare("Hand")){
                // this makes the palm segment appear a bit longer, making it more similar to the 'dave' model
                cylindroids.push_back(std::make_shared<Cylindroid>(program, s.radius1, s.radius1_y, s.radius2, s.radius2_y, s.length + 5));
            } else {
                cylindroids.push_back(std::make_shared<Cylindroid>(program, s.radius1, s.radius1_y, s.radius2, s.radius2_y, s.length));
            }
#endif
        }
    }
}

Vector3 rgb_color_for(std::string& name)
{
    /// Qualitative #6
    /// http://colorbrewer2.org

    if(name.size()<5)
        return Vector3(0,0,0); ///< palm
    // int phalanx = strtod( *name.end() );
    if(name.at(4)=='T') ///< HandThumb*
        return Vector3(77,175,74);
    if(name.at(4)=='I') ///< HandIndex*
        return Vector3(228,26,28);
    if(name.at(4)=='R') ///< HandRing*
        return Vector3(55,126,184);
    if(name.at(4)=='M') ///< HandMiddle*
        return Vector3(152,78,163);
    if(name.at(4)=='P') ///< HandPinky*
        return Vector3(255,127,0);
    return Vector3(0,0,0);
}

void Cylinders_renderer::render()
{
    program.bind();
    for(int i=0; i<cylindroids.size(); i++){
        const Segment &s = cylinders->getSegments()[i];
        const Matrix4 &m = s.joint->getGlobalTransformation();
        std::string jname = s.joint->getName();
        
        ///-----------------------------------
        /// @note ugly, we could do this in the init-geometry
        //Vector3 color(1,0,0); ///< red
        //Vector3 color(1,1,1); ///< white/grey
        //Vector3 color(183/255.0f,213/255.0f,231/255.0f); ///< blue/grey-ish
        Vector3 color(137/255.0f,192/255.0f,226/255.0f); ///< blue-ish
        if(mode==SKINNY)
            color = rgb_color_for(jname)/255.0;
        ///-----------------------------------

        size_t id = cylinders->getSkeleton()->getID(jname);
        cylindroids[i]->render(program, m, id, color);
        
// #define PALM_ONLY
#ifdef PALM_ONLY
        break;
#endif 
    }
    program.release();
}

