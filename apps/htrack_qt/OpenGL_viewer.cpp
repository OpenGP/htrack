#include "OpenGL_viewer.h"

///--- External
#include <QContextMenuEvent>
#include <QWindow>

#ifdef WITH_QGLVIEWER
#include <QGLViewer/qglviewer.h>
#include <QGLViewer/camera.h>
#include <QGLViewer/manipulatedCameraFrame.h>
#endif

#include "util/OpenGL32Format.h"
#include "util/eigen_opengl_helpers.h"

///--- Internal
#include "tracker/OpenGL/QuadRenderer/QuadRenderer.h"
#include "tracker/OpenGL/CylindersRenderer/Cylinders_renderer.h"
#include "tracker/OpenGL/KinectDataRenderer/KinectDataRenderer.h"
#include "tracker/OpenGL/DebugRenderer/DebugRenderer.h"
#include "tracker/Data/TextureColor8UC3.h"
#include "tracker/Data/TextureDepth16UC1.h"

#include "tracker/Data/Camera.h"
#include "tracker/Worker.h"
#include "util/mylogger.h"

/// IMREAD TEST
#include "opencv2/highgui/highgui.hpp"
#include "tracker/HandFinder/HandFinder.h"
#include "tracker/DataStructure/SkeletonSerializer.h"

// #define RESULTS

OpenGL_viewer::OpenGL_viewer(Worker *_worker) : OpenGL_viewer_Superclass(OpenGL32Format(), NULL), worker(_worker), _camera(_worker->camera){
    CHECK_NOTNULL(worker);
    CHECK_NOTNULL(worker->cylinders);
    CHECK_NOTNULL(worker->skeleton);
    CHECK_NOTNULL(worker->camera);

    color_ch = std::make_shared<QuadRenderer>(QuadRenderer::Color, _camera);
    depth_ch = std::make_shared<QuadRenderer>(QuadRenderer::Depth, _camera);
    mrenderer = std::make_shared<Cylinders_renderer>(worker->cylinders);
    srenderer = std::make_shared<Cylinders_renderer>(worker->cylinders);
    kinect_renderer = std::make_shared<KinectDataRenderer>();

#ifdef BROKEN_CONSTANT_ASPECT_RATIO
    /// @see http://stackoverflow.com/questions/16801237/how-to-maintain-specific-height-to-width-ratio-of-widget-in-qt5
    /// @see http://doc.qt.digia.com/qq/qq04-height-for-width.html
    QSizePolicy policy(QSizePolicy::Preferred, QSizePolicy::Preferred);
    policy.setHeightForWidth(true);
    setSizePolicy(policy);
#else
    setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
#endif

    ///--- Create the contextual menu
    contextual = new QMenu(this);
    auto add_entry = [=](const char* name, bool _default) {
        QAction* action = contextual->addAction(name);
        action->setCheckable(true);
        action->setChecked(_default);
        return action;
    };
    draw_model = add_entry("draw_model", true);
    draw_cloud = add_entry("draw_cloud", true);
    draw_skeleton = add_entry("draw_skeleton", false);
    contextual->addSeparator();
    draw_sensor_color = add_entry("draw_sensor_color", true);
    draw_sensor_depth = add_entry("draw_sensor_depth", true);
    contextual->addSeparator();
    disable_depth_test = add_entry("disable_depth_test", false);
    use_kinect_camera = add_entry("use_kinect_camera", false);
    draw_color_fullscreen = add_entry("draw_color_fullscreen", false);
    draw_depth_fullscreen = add_entry("draw_depth_fullscreen", false);
    contextual->addSeparator();
    draw_debug = add_entry("draw_debug", true);
    clear_debug = contextual->addAction("clear_debug");
    wristband_colormap = add_entry("wristband_colormap", true);

#ifdef RESULTS
    disable_depth_test->setChecked(true);
    draw_model->setChecked(false);
    draw_skeleton->setChecked(true);
    draw_sensor_color->setChecked(false);
    draw_sensor_depth->setChecked(false);
    use_kinect_camera->setChecked(true);
#endif

    ///--- Connect single shot events
    connect(clear_debug, &QAction::triggered, &(DebugRenderer::instance()), &DebugRenderer::clear);
}

OpenGL_viewer::~OpenGL_viewer(){
    worker->cleanup_graphic_resources();
}

void OpenGL_viewer::contextMenuEvent(QContextMenuEvent* event)
{
#ifdef __unix__
    // MS: my OS already uses alt+click, but not ctrl+click
    if(event->modifiers().testFlag(Qt::ControlModifier))
#else
    if(event->modifiers().testFlag(Qt::AltModifier))
#endif
    {
        contextual->exec(event->globalPos());
        updateGL();
    }
}


/// @warning QGLViewer pollutes OpenGL status. Resetting it here
void OpenGL_viewer::initializeGL(){
    LOG(INFO) << "OpenGL_viewer::initializeGL() OpenGL" << this->format().majorVersion() << "." << this->format().minorVersion();
    glClearColor(1.0, 1.0, 1.0, 0.0);
    glEnable(GL_DEPTH_TEST);

#ifdef WITH_QGLVIEWER
    camera()->frame()->setSpinningSensitivity(100); ///<<< Disable spin

    /// Setup default camera
    reset_camera();

    /// Bindings @see QGLViewer::setDefaultMouseBindings()
    /// Extra behavior in this->mouseDoubleClickEvent()
    {
        /// Disable double click to align scene
        setMouseBinding(Qt::NoModifier, Qt::LeftButton, NO_CLICK_ACTION, true);   /// ALIGN_CAMERA
        setMouseBinding(Qt::ShiftModifier, Qt::RightButton, NO_CLICK_ACTION);     /// RAP_FROM_PIXEL
        setMouseBinding(Qt::NoModifier, Qt::MiddleButton, NO_CLICK_ACTION, true); /// ZOOM_TO_FIT
    }
#endif

#ifdef WITH_GLEW
    initialize_glew();
#endif

    ///--- Compile/initialize shaders
    mrenderer->init(Cylinders_renderer::NORMAL);
    srenderer->init(Cylinders_renderer::SKINNY);
    color_ch->init();
    depth_ch->init();
    kinect_renderer->init(_camera);
    CHECK_ERROR_GL();

    ///--- Initialize other graphic resources
    this->makeCurrent();
    worker->init_graphic_resources();

    ///--- Setup with data from worker
    color_ch->setup(worker->sensor_color_texture->texid());
    depth_ch->setup(worker->sensor_depth_texture->texid());
    kinect_renderer->setup(worker->sensor_color_texture->texid(), worker->sensor_depth_texture->texid());
}

void OpenGL_viewer::mouseDoubleClickEvent(QMouseEvent* e) {
#ifdef WITH_QGLVIEWER
    /// MeshLAB like double click action
    {
        /// Modified version of "RAP_FROM_PIXEL"
        if (!camera()->setPivotPointFromPixel(e->pos()))
            return; // do nothing
        camera()->setSceneCenter( camera()->pivotPoint() );
        /// Stolen from "centerScene"
        camera()->frame()->projectOnLine(sceneCenter(), camera()->viewDirection());
        setVisualHintsMask(1);
        update();
    }
#endif
}

void OpenGL_viewer::reset_camera() {
#ifdef WITH_QGLVIEWER
    /// Default OpenGL camera (right-hand coordinate system)
    camera()->setType(qglviewer::Camera::ORTHOGRAPHIC);
    //camera()->setType(qglviewer::Camera::PERSPECTIVE);
    camera()->setPosition(qglviewer::Vec(0,0,0));
    camera()->lookAt(qglviewer::Vec(0,0,1));
    camera()->setUpVector(qglviewer::Vec(0,1,0));

    /// Setup camera zNear/zFar
    camera()->setSceneCenter(qglviewer::Vec(0,0,300/*mm*/));
    camera()->setSceneRadius(200 /*mm*/);
    camera()->showEntireScene();
#endif
}

void OpenGL_viewer::reload_model(){
    /// This recomputes segments lengths
    worker->cylinders->recomputeLengths();
    /// This resets the "local" transformations
    //worker->skeleton->resetPosture(true);
    /// This accepts the new joint translations
    worker->skeleton->setInitialTranslations();
    /// This generates VBO from segments
    mrenderer->init_geometry(); // thick model
    srenderer->init_geometry(); // skinny model
    /// After change, show what's happened
    this->updateGL();
}


int OpenGL_viewer::width() {
    qreal r = window()->windowHandle()->devicePixelRatio();
    return QGLWidget::width()*r;
}

int OpenGL_viewer::height() {
    qreal r = window()->windowHandle()->devicePixelRatio();
    return QGLWidget::height()*r;
}

void OpenGL_viewer::paintGL() {
    /// Mostly for results generation
    glEnable(GL_MULTISAMPLE);

    glEnable(GL_DEPTH_TEST);
    glViewport(0,0,this->width(),this->height());
    if(transparent_background)
        glClearColor(1,1,1,0); ///< transparent background
    else
        glClearColor(1,1,1,1);

    ///--- Clear the screen
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    ///--- Setup camera (Model View Projection)
    bool use_kinect_camera = this->use_kinect_camera->isChecked();

    Eigen::Matrix4f view_projection;
    Eigen::Matrix4f view;
#ifndef WITH_QGLVIEWER
    use_kinect_camera = true;
#endif
    if(!use_kinect_camera){
#ifdef WITH_QGLVIEWER
        /// This gives me OpenGL2 style matrices (right-hand system)
        /// @see reset_camera()
        camera()->getModelViewProjectionMatrix(view_projection.data());
        camera()->getModelViewMatrix(view.data());
        /// !!! set a left-hand coordinate system (remove x-coord flip of lookAt)
        view_projection.col(0) = -view_projection.col(0);
        view.col(0) = -view.col(0);
#else
        exit(EXIT_FAILURE);
#endif
    }
    else{
        view_projection = _camera->view_projection_matrix();
        view = _camera->view_matrix();
    }
    // std::cout << "MVP\n" << MVP << std::endl;
    // std::cout << "MVP" << Camera(QVGA).view_projection_matrix() << std::endl;

    if (draw_debug->isChecked()){
        DebugRenderer::instance().set_uniform("view_projection",view_projection);
        DebugRenderer::instance().render();
        CHECK_ERROR_GL();
    }

    if(use_kinect_camera)
    {
        if ( draw_color_fullscreen->isChecked() )
        {
            glViewport(0,0,this->width(),this->height());
            color_ch->render();
        }
        else if( draw_depth_fullscreen->isChecked() )
        {
            glViewport(0,0,this->width(),this->height());
            depth_ch->render();
        }
    }

    ///--- draw model
    if (draw_model->isChecked()){
        mrenderer->set_uniform("view",view);
        mrenderer->set_uniform("view_projection",view_projection);
        mrenderer->render();
        CHECK_ERROR_GL();
    }

    ///--- draw skeleton (when it's shown transparency)
    if (draw_skeleton->isChecked() && disable_depth_test->isChecked()){
        srenderer->set_uniform("view",view);
        srenderer->set_uniform("view_projection",view_projection);
        srenderer->render();
        CHECK_ERROR_GL();
    }

    ///--- draw kinect point cloud in shader
    if(draw_cloud->isChecked())
    {
        if(wristband_colormap->isChecked() && worker->handfinder->wristband_found())
        {
            kinect_renderer->enable_colormap(true);
            kinect_renderer->set_zNear(worker->handfinder->wristband_center()[2] - 150);
            kinect_renderer->set_zFar(worker->handfinder->wristband_center()[2] + 150);
        }

        kinect_renderer->set_uniform("view_projection",view_projection);
        kinect_renderer->render();
        CHECK_ERROR_GL();
    }

    ///--- draw skeleton (when it's shown on top)
    if (draw_skeleton->isChecked() && !disable_depth_test->isChecked()){
        glClear(GL_DEPTH_BUFFER_BIT);
        srenderer->set_uniform("view",view);
        srenderer->set_uniform("view_projection",view_projection);
        srenderer->render();
        CHECK_ERROR_GL();
    }

    ///--- channels overlay
    {
        ///--- Settings
        const int width = this->width()/5;
        const float ratio = 240.0/320.0;
        int height = width*ratio;
        const int pad = 10;

        if(draw_sensor_color->isChecked()){
            glViewport(pad,this->height()-height-pad,width,height); ///< Top left
            color_ch->render();
            CHECK_ERROR_GL();
        }

        if(draw_sensor_depth->isChecked()){
            glViewport(this->width()-width-pad,this->height()-height-pad,width,height); ///< Top right
            depth_ch->render();
            CHECK_ERROR_GL();
        }
    }

    glDisable(GL_MULTISAMPLE);
    CHECK_ERROR_GL();
}
