#pragma once
#include "util/gl_wrapper.h"
#include "tracker/ForwardDeclarations.h"
#include "tracker/Types.h"
#include "OffscreenRender/OffscreenRenderer.h"
#include "Data/DataFrame.h"
#include "Energy/JointLimits.h"
#include "Energy/Damping.h"
#include "Energy/Collision.h"
#include "Energy/PoseSpace.h"
#include "Energy/Fitting.h"
#include "Energy/Fitting/TrackingMonitor.h"
#include "Energy/Wristband.h"
#include "Energy/Temporal.h"

/// @note do not construct more than one instance of this class
class Worker{
/// @{ Settings
public:
   struct Settings{
       int termination_max_iters = 7;
       int termination_max_rigid_iters = 1;
   } _settings;
   Settings*const settings = &_settings;
/// @}

/// @{ to refresh UI upon tracking changes
private:
   QGLWidget* glarea = NULL;
public:
   void bind_glarea(QGLWidget* glarea){ this->glarea = glarea; }
   void updateGL();
/// @}

public:
   Camera* camera = NULL;
   SkeletonSerializer* skeleton = NULL;
   Cylinders* cylinders = NULL;
   DataFrame current_frame = DataFrame(-1);

/// @{ OpenGL textures that store the sensor data
   DepthTexture16UC1* sensor_depth_texture = NULL;
   ColorTexture8UC3* sensor_color_texture = NULL;
/// @}

/// @{ Energies that assemble the optimization
    energy::Fitting E_fitting;
    energy::Temporal E_temporal;
    energy::Damping E_damping;
    energy::JointLimits E_limits;
    energy::Collision E_collision;
    energy::PoseSpace E_pose;
    energy::Wristband E_wristband;

    HandFinder* handfinder = NULL;
    TrivialDetector* trivial_detector = NULL;
    OffscreenRenderer offscreenrend;
    TrackingMonitor monitor;
/// @}

public:
   Worker(Camera *camera);
   ~Worker();
   void init_graphic_resources(); ///< not in constructor as needs valid OpenGL context
   void cleanup_graphic_resources();

/// @{
public:
   TrackingError track(DataFrame& frame, bool rigid, bool eval_error);
   bool track_till_convergence(DataFrame& frame);
/// @}
};
