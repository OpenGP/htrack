#include <QApplication>
#include <QGLViewer/qglviewer.h>

class Viewer : public QGLViewer {
    void draw() {
        /// @note this would not work in Mac as there is no compatibility profile
        /// and this is OpenGL2 stuff.
        const float nbSteps = 200.0;
        glBegin(GL_QUAD_STRIP);
        for (int i=0; i<nbSteps; ++i) {
            const float ratio = i/nbSteps;
            const float angle = 21.0*ratio;
            const float c = cos(angle);
            const float s = sin(angle);
            const float r1 = 1.0 - 0.8f*ratio;
            const float r2 = 0.8f - 0.8f*ratio;
            const float alt = ratio - 0.5f;
            const float nor = 0.5f;
            const float up = sqrt(1.0-nor*nor);
            glColor3f(1.0-ratio, 0.2f , ratio);
            glNormal3f(nor*c, up, nor*s);
            glVertex3f(r1*c, alt, r1*s);
            glVertex3f(r2*c, alt+0.05f, r2*s);
        }
        glEnd();
    }
    void init() {
        restoreStateFromFile();
    }
};

int main(int argc, char* argv[]){
    QApplication app(argc, argv);
    Viewer viewer;
    viewer.setWindowTitle("helloworld_qglviewer");
    viewer.show();;
    return app.exec();
}
