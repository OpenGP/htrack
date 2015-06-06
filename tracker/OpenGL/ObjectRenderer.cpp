#include "tracker/OpenGL/ObjectRenderer.h"

// had to separate declaration from implementation due to compilation errors on linux
void ObjectRenderer::set_uniform(const char* name, const Eigen::Matrix4f& value){
    assert(program.isLinked());
    program.bind();
    GLuint id = glGetUniformLocation(program.programId(),name);
    if(id==-1)
        printf("!!!WARNING: shader '%d' does not contain uniform variable '%s'\n", program.programId(), name);
    glUniformMatrix4fv(id, 1, GL_FALSE, value.data());
    program.release();
}

void ObjectRenderer::set_uniform(const char* name, const Eigen::Matrix3f& value)
{
    assert(program.isLinked());
    program.bind();
    GLuint id = glGetUniformLocation(program.programId(),name);
    if(id==-1)
        printf("!!!WARNING: shader '%d' does not contain uniform variable '%s'\n", program.programId(), name);
    glUniformMatrix3fv(id, 1, GL_FALSE, value.data());
    program.release();
}

void ObjectRenderer::set_uniform(const char* name, float value)
{
    assert(program.isLinked());
    program.bind();
    GLuint id = glGetUniformLocation(program.programId(),name);
    if(id==-1)
        printf("!!!WARNING: shader '%d' does not contain uniform variable '%s'\n", program.programId(), name);
    glUniform1f(id, value);
    program.release();    
}

void ObjectRenderer::set_texture(const char* name, int value)
{
    assert(program.isLinked());
    program.bind();
        glUniform1i(glGetUniformLocation(program.programId(), name), value /*GL_TEXTURE_{VALUE}*/);
    program.release();
}

void ObjectRenderer::set_uniform_uint(const char* name, unsigned int value)
{
    assert(program.isLinked());
    program.bind();
	GLuint id = glGetUniformLocation(program.programId(), name);
        glUniform1ui(id, value);
    program.release();
}
