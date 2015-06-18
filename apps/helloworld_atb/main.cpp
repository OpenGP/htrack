#include <iostream>
#include <QDebug>
#include <QApplication>
#include <QGLWidget>
#include <QOpenGLVertexArrayObject>

#include "../htrack_atb/AntTweakBarEventFilter.h"
#include "tracker/TwSettings.h"

float v1;
float g1_v1;
float g1_v2;

#include "AntTweakBar.h"


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
        tw_settings->tw_init(this->width(), this->height());
        tw_settings->tw_add(v1, "v1", "min=.5 max=50 step=0.4");
        tw_settings->tw_add(g1_v1, "g1_v1", "group=g1");
        tw_settings->tw_add(g1_v2, "g1_v2", "group=g1");
    }

    void paintGL() {
        glClearColor(0,1,0,1); ///< green
        glClear(GL_COLOR_BUFFER_BIT);
        tw_settings->tw_draw();
        // TwWindowSize(this->width(), this->height());
        TwDraw();
    }
};

int main(int argc, char* argv[]){
    QApplication app(argc, argv);
    GLWidget glarea;
    glarea.show();
    return app.exec();
}
