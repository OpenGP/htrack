#include <iostream>
#include <QDebug>
#include <QApplication>
#include <QGLWidget>
#include <QOpenGLVertexArrayObject>

#include "../htrack_atb/AntTweakBarEventFilter.h"
#include "AntTweakBar.h"
TwBar* _bar = NULL;

float v1;
float g1_v1;
float g1_v2;


/// Format class to enable OpenGL4 core profile
class OpenGLFormat : public QGLFormat{
public:
    OpenGLFormat(){
        setVersion(3,2);
        setProfile(QGLFormat::CoreProfile);
        setSampleBuffers(false); ///< no anti-aliasing
        // setSamples(1); ///< no anti-aliasing
    }
};

class GLWidget : public QGLWidget{
    QOpenGLVertexArrayObject vao;
public:
    GLWidget():QGLWidget(OpenGLFormat()){
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
        TwWindowSize(this->width(), this->height());
        _bar = TwNewBar ("Settings");
        TwAddVarRW(_bar, "p", TW_TYPE_FLOAT, &v1, "min=.5 max=50 step=0.4");
        TwAddVarRW(_bar, "V1", TW_TYPE_FLOAT, &g1_v1, " group=g1 ");
        TwAddVarRW(_bar, "V2", TW_TYPE_FLOAT, &g1_v2, " group=g1 ");
    }

    void paintGL() {
        glClearColor(0,1,0,1); ///< green
        glClear(GL_COLOR_BUFFER_BIT);
        TwWindowSize(this->width(), this->height());
        TwDraw();
    }
};

int main(int argc, char* argv[]){
    QApplication app(argc, argv);
    GLWidget glarea;
    glarea.show();
    return app.exec();
}
