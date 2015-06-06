#include <iostream>
#include <QDebug>
#include <QApplication>
#include <QGLWidget>
#include <QOpenGLWidget>
#include <QOpenGLVertexArrayObject>

#include "gui/AntTweakBarEventFilter.h"
#include "AntTweakBar.h"
TwBar* _bar = NULL;

float v1;
float g1_v1;
float g1_v2;

class GLWidget : public QOpenGLWidget{
    QOpenGLVertexArrayObject vao;
public:
    GLWidget():QOpenGLWidget(){
        std::cout << "OpenGL " << this->format().majorVersion() << "." << this->format().minorVersion() << std::endl;
        this->installEventFilter( new AntTweakBarEventFilter(this) ); ///< all actions pass through filter
    }
    ~GLWidget(){
        TwTerminate();
    }
    void initializeGL(){
        bool success = vao.create();
        if(!success) exit(EXIT_FAILURE);
        vao.bind();
        TwInit(TW_OPENGL_CORE, NULL);
        TwWindowSize(this->width()*2, this->height()*2);
        _bar = TwNewBar ("Settings");
        TwAddVarRW(_bar, "p", TW_TYPE_FLOAT, &v1, "min=.5 max=50 step=0.4");
        TwAddVarRW(_bar, "V1", TW_TYPE_FLOAT, &g1_v1, " group=g1 ");
        TwAddVarRW(_bar, "V2", TW_TYPE_FLOAT, &g1_v2, " group=g1 ");
        glViewport(0, 0, this->width(), this->height());
    }

    void paintGL() {
        glViewport(0,0,this->width(), this->height());
        glClearColor(0,1,0,1); ///< green
        glClear(GL_COLOR_BUFFER_BIT);
        TwDraw();
    }
};

int main(int argc, char* argv[]){
    QApplication app(argc, argv);
    
    QSurfaceFormat format;
    format.setVersion(4, 3);
    format.setProfile(QSurfaceFormat::CoreProfile);
    format.setDepthBufferSize(24);
    format.setStencilBufferSize(8);
    QSurfaceFormat::setDefaultFormat(format);
    
    GLWidget glarea;
    glarea.show();
    return app.exec();
}
