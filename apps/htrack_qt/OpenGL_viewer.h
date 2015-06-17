#pragma once
#include "tracker/ForwardDeclarations.h"

#include "util/gl_wrapper.h"
#include <memory>
#include <QMenu>

#ifdef WITH_QGLVIEWER
#include <QGLViewer/qglviewer.h>
typedef QGLViewer OpenGL_viewer_Superclass;
#else
#include <QGLWidget>
typedef QGLWidget OpenGL_viewer_Superclass;
#endif

class OpenGL_viewer : public OpenGL_viewer_Superclass {
    Q_OBJECT
private:
    Worker*const worker;
    Camera*const _camera;
    std::shared_ptr<QuadRenderer>       color_ch;
    std::shared_ptr<QuadRenderer>       depth_ch;
    std::shared_ptr<Cylinders_renderer> mrenderer;
    std::shared_ptr<Cylinders_renderer> srenderer;
    std::shared_ptr<KinectDataRenderer> kinect_renderer;
    
/// @{
private:
    friend class Main_window;
    bool transparent_background = false;
/// @} 

/// @{ contextual menu
protected:
    void contextMenuEvent(QContextMenuEvent *event);
protected:
    QMenu* contextual;
    QAction* draw_cloud;
    QAction* draw_skeleton;
    QAction* draw_debug;
    QAction* use_kinect_camera;
    QAction* draw_model;    
    QAction* draw_sensor_color;
    QAction* draw_sensor_depth;
    QAction* draw_color_fullscreen;
    QAction* draw_depth_fullscreen;
    QAction* clear_debug;    
    QAction* disable_depth_test;
    QAction* wristband_colormap;
/// @}

public:
    /// default window size
//    QSize sizeHint() const { return QSize(640, 480); }
    QSize sizeHint() const { return QSize(800, 600); }
    OpenGL_viewer(Worker*worker);
    ~OpenGL_viewer();
        
public slots:    
    void reset_camera();
    /// Reload for when parameters of hand are modified
    void reload_model();
public:
    void initializeGL(); /// Inits the whole scene (shaders, buffers, etc)
    void paintGL(); /// Draws the whole scene
    int width();
    int height();
protected:
    void mouseDoubleClickEvent(QMouseEvent *e);
};
