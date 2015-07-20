#include <iostream>
#include <QDebug>
#include <QApplication>
#include <QDir>

#include "util/gl_wrapper.h"
#include "util/OpenGL32Format.h"
#include <QGLWidget>
#include "AntTweakBarEventFilter.h"

#include "tracker/ForwardDeclarations.h"
#include "tracker/HandFinder/HandFinder.h"
#include "tracker/Sensor/Sensor.h"
#include "tracker/Data/Camera.h"
#include "tracker/Calibration/Calibration.h"
#include "tracker/OpenGL/KinectDataRenderer/KinectDataRenderer.h"
#include "tracker/OpenGL/CylindersRenderer/Cylinders_renderer.h"
#include "tracker/Tracker.h"

class GLWidget : public QGLWidget{
public:
    Worker*const worker;
    DataStream*const datastream;
    SolutionStream*const solutions;

    Camera*const _camera;
    KinectDataRenderer kinect_renderer;
    Cylinders_renderer mrenderer;

public:
    GLWidget(Worker* worker, DataStream * datastream, SolutionStream * solution):
        QGLWidget(OpenGL32Format()),
        worker(worker),
        datastream(datastream),
        solutions(solutions),
        _camera(worker->camera),
        mrenderer(worker->cylinders)
    {
        std::cout << "Started OpenGL " << this->format().majorVersion() << "." << this->format().minorVersion() << std::endl;
        this->installEventFilter( new AntTweakBarEventFilter(this) ); ///< all actions pass through filter
    }

    ~GLWidget(){
        worker->cleanup_graphic_resources();
        tw_settings->tw_cleanup();
    }

    void initializeGL(){
        std::cout << "GLWidget::initializeGL()" << std::endl;
        initialize_glew();
        tw_settings->tw_init(this->width(), this->height()); ///< FIRST!!

        glEnable(GL_DEPTH_TEST);

        kinect_renderer.init(_camera);
        mrenderer.init(Cylinders_renderer::NORMAL);
        mrenderer.init_geometry(); // thick model

        ///--- Initialize other graphic resources
        this->makeCurrent();
        worker->init_graphic_resources();

        ///--- Setup with data from worker
        kinect_renderer.setup(worker->sensor_color_texture->texid(), worker->sensor_depth_texture->texid());
    }

    void paintGL() {
        glViewport(0,0,this->width(),this->height());
        glClearColor(1,1,1,1);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        ///--- Rendering
        Eigen::Matrix4f view_projection = _camera->view_projection_matrix();
        Eigen::Matrix4f view = _camera->view_matrix();

        if(worker->handfinder->wristband_found())
        {
            kinect_renderer.enable_colormap(true);
            kinect_renderer.set_zNear(worker->handfinder->wristband_center()[2] - 150);
            kinect_renderer.set_zFar(worker->handfinder->wristband_center()[2] + 150);
        }
        kinect_renderer.set_uniform("view_projection",view_projection);
        kinect_renderer.render();

        mrenderer.set_uniform("view",view);
        mrenderer.set_uniform("view_projection",view_projection);
        mrenderer.render();

        tw_settings->tw_draw();
    }

    void reload_model(){
        /// This recomputes segments lengths
        worker->cylinders->recomputeLengths();
        /// This accepts the new joint translations
        worker->skeleton->setInitialTranslations();
        /// This generates VBO from segments
        mrenderer.init_geometry();
        /// After change, show what's happened
        this->updateGL();
    }

private:
    void keyPressEvent(QKeyEvent *event){
        GLWidget* qglviewer = this;
        switch(event->key()){
            case Qt::Key_Escape:
                this->close();
                break;
            case Qt::Key_1:
                // make_hand_thinner();
                worker->skeleton->scaleWidth(-5);
                Calibration::update_radius_helper(worker->cylinders, worker->skeleton);
                qglviewer->reload_model();
                break;
            case Qt::Key_2:
                // make_hand_wider();
                worker->skeleton->scaleWidth(5);
                Calibration::update_radius_helper(worker->cylinders, worker->skeleton);
                qglviewer->reload_model();
                break;
            case Qt::Key_3:
                // make_hand_shorter();
                worker->skeleton->scaleHeight(-1);
                Calibration::update_radius_helper(worker->cylinders, worker->skeleton);
                qglviewer->reload_model();
                break;
            case Qt::Key_4:
                // make_hand_longer();
                worker->skeleton->scaleHeight(1);
                Calibration::update_radius_helper(worker->cylinders, worker->skeleton);
                qglviewer->reload_model();
                break;
            case Qt::Key_5:
                // make_hand_smaller();
                worker->skeleton->scale(0.99f);
                qglviewer->reload_model();
                break;
            case Qt::Key_6:
                // make_hand_bigger();
                worker->skeleton->scale(1.01f);
                qglviewer->reload_model();
                break;
            default:
                    QGLWidget::keyPressEvent(event);
        }
    }
};

int main(int argc, char* argv[]){
    Q_INIT_RESOURCE(shaders); ///< http://qt-project.org/wiki/QtResources
    QApplication app(argc, argv);
    std::cout << "htrack starting" << std::endl;
    std::cout << "--Execution path: " << QDir::currentPath().toStdString() << std::endl;

#if defined(SOFTKIN) && !defined(__APPLE__)
    Camera camera(Intel, 60 /*FPS*/);
    SensorSoftKin sensor(&camera);
#endif

#if defined(DEPTHSENSEGRABBER) && !defined(__APPLE__)
    Camera camera(Intel, 60 /*FPS*/);
    SensorDepthSenseGrabber sensor(&camera);
#endif

#if defined(OPENNI)
    Camera camera(QVGA, 60 /*FPS*/);
    // Camera camera(QVGA, 30 /*FPS*/);
    SensorOpenNI sensor(&camera);
#endif

#if defined(REALSENSE)
    Camera camera(QVGA, 60 /*FPS*/);
    SensorRealSense sensor(&camera);
#endif

    DataStream datastream(&camera);
    SolutionStream solutions;

    Worker worker(&camera);
    GLWidget glarea(&worker, &datastream, &solutions);
    glarea.resize(640*2,480*2); ///< force resize
    worker.bind_glarea(&glarea); ///< TODO: can we avoid this?

    ///--- Load calibration
    Calibration(&worker).autoload();
    // glarea->reload_model(); ///< TODO


    glarea.show(); ///< calls GLWidget::InitializeGL

    Tracker tracker(&worker,camera.FPS());
    tracker.sensor = &sensor;
    tracker.datastream = &datastream;
    tracker.solutions = &solutions;

    ///--- Starts the tracking
    tracker.toggle_tracking(true);

    return app.exec();
}
