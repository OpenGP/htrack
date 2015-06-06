#include <iostream>
#include <QApplication>
#include <QGLWidget>
#include <QOpenGLVertexArrayObject>

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
    }
     void initializeGL(){
        bool success = vao.create();
        if(!success) exit(EXIT_FAILURE);
        vao.bind();
    }

    void paintGL() {
        glClearColor(0,1,0,1); ///< green
        glClear(GL_COLOR_BUFFER_BIT);
    }
};


int main(int argc, char* argv[]){
    QApplication app(argc, argv);
    GLWidget glarea;
    glarea.show();
    return app.exec();
}
