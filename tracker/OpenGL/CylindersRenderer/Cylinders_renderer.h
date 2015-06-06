#pragma once
#include "tracker/Legacy/geometry/Cylinders.h"
#include "tracker/OpenGL/ObjectRenderer.h"
#include <memory>

class Cylindroid;

class Cylinders_renderer : public ObjectRenderer{
public:
    enum SHADERMODE{ NORMAL, SKINNY, FRAMEBUFFER };
private:
    typedef std::shared_ptr<Cylindroid> CylindroidPtr; 
    std::vector<CylindroidPtr> cylindroids; 
    Cylinders *cylinders;    
    Scalar segments_size=3; ///< default width of cylinders in "SKINNY" mode
    SHADERMODE mode;
    
public:
    Cylinders_renderer(Cylinders *c);
    /// Allows static init "static Cylinders_renderer a(c, mode, matrix);
    Cylinders_renderer(Cylinders *c, SHADERMODE mode, const Eigen::Matrix4f& view_projection);

    void setLighting(bool){} ///TODO
    void init(SHADERMODE mode=NORMAL);
    void init_geometry();
    void render();
    
private:
    /// Setup only allowed in constructor!!
    void setup(Cylinders *c);    
};
