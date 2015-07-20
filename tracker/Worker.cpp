#include "Worker.h"
#include "util/gl_wrapper.h"
#include "util/tictoc.h"

#include <QElapsedTimer>
#include <QGLWidget>

#include "tracker/ForwardDeclarations.h"
#include "tracker/Sensor/Sensor.h"
#include "tracker/DataStructure/SkeletonSerializer.h"
#include "tracker/Data/TextureColor8UC3.h"
#include "tracker/Data/TextureDepth16UC1.h"
#include "tracker/HandFinder/HandFinder.h"
#include "tracker/Energy/Energy.h"
#include "tracker/TwSettings.h"

void Worker::updateGL(){ if(glarea!=NULL) glarea->updateGL(); }

Worker::Worker(Camera *camera){
    this->camera = camera;
    this->skeleton = SkeletonSerializer::leftHand2<SkeletonSerializer>();
    this->skeleton->reset_hand();
    this->skeleton->generateIDs();
    cylinders = new Cylinders(skeleton);
}

/// @note any initialization that has to be done once GL context is active
void Worker::init_graphic_resources(){
    offscreenrend.init(camera, cylinders);
    sensor_color_texture = new ColorTexture8UC3(camera->width(), camera->height());
    sensor_depth_texture = new DepthTexture16UC1(camera->width(), camera->height());

    tw_settings->tw_add(settings->termination_max_iters,"#iters","group=Tracker");
    tw_settings->tw_add(settings->termination_max_rigid_iters,"#iters (rigid)","group=Tracker");
    tw_settings->tw_add(monitor.settings->moving_window_size,"failure confidence","group=Tracker");


    ///--- Initialize the energies modules
    using namespace energy;
    trivial_detector = new TrivialDetector(camera, &offscreenrend, skeleton);
    handfinder = new HandFinder(camera, trivial_detector);
    E_fitting.init(this);
    E_wristband.init(camera,skeleton,handfinder);
    E_limits.init(skeleton);
    E_collision.init(this);
    E_pose.init();
    E_temporal.init(skeleton);
    E_damping.init(skeleton);
}

void Worker::cleanup_graphic_resources(){
    /// TODO verify texture de-allocations are correct!
    delete sensor_color_texture;
    delete sensor_depth_texture;
    E_fitting.cleanup();
}

Worker::~Worker(){
    delete trivial_detector;
    delete handfinder;
    delete skeleton;
    delete cylinders;
}

bool Worker::track_till_convergence(DataFrame& frame) {
    using namespace energy;
    TrackingError error = {0.0f,0.0f};
    for (int i=0; i < settings->termination_max_iters; ++i) {
        // tic(t_iteration);
        bool eval_error = (i==settings->termination_max_iters-1);
        bool rigid_only = (i<settings->termination_max_rigid_iters);
        error = track(frame,rigid_only, eval_error);
        // std::cout << toc(t_iteration) << " ";
#if 0
        /// @note Do not waste time in this routine!!
        ///--- Draw onto screen
        ::glarea->paintGL();
        ::glarea->swapBuffers();
#endif
    }
    // std::cout << std::endl;

    return monitor.is_failure_frame(error._3D, error._2D);
}

TrackingError Worker::track(DataFrame& frame, bool rigid_only, bool eval_error){
    // LOG(INFO) << "iter: " << iteration << max_rigid_iters << (iteration < max_rigid_iters) << rigid_only;

    // LOG(INFO) << "----------------------------------------------------";
    // mDebug() << "Worker::track("<<frame.id<<")";
    // TICTOC_SCOPE(timer,"Worker::track");

    ///--- Classify the sensor pixels to produce Sensor silhouette
    handfinder->binary_classification(frame);
    if(!handfinder->has_useful_data()){
        // LOG(INFO) << "no useful data";
        skeleton->reset_hand();
        return TrackingError::infinity();
    }

#if 0
    /// TODO: find out how this needs to be fixed
    ///--- Set previous frames from solution stream
    if(solutions->isValid() && frame.id >= 2){
        solution_queue.set(frame.id-1, solutions->get(frame.id-1));
        solution_queue.set(frame.id-2, solutions->get(frame.id-2));
    }
#endif

    ///--- Current pose
    std::vector<float> _thetas = skeleton->getCurrentParameters();

    ///--- Render hand model
    offscreenrend.render_offscreen();

    ///--- Combine linear systems
    LinearSystem system(num_thetas);

    ///--- Serialize matrices for jacobian computation
    skeleton->update();

    ///--- Optimization phases
    float push_error=0, pull_error=0;
    E_fitting.track(frame, system, rigid_only, eval_error, push_error, pull_error); ///<!!! MUST BE FIRST CALL
    E_collision.track(system);
    E_temporal.track(system, frame);
    E_wristband.track(system);
    E_limits.track(system, _thetas);
    E_damping.track(system);

    if(!rigid_only){
        E_pose.track(system,_thetas); ///<!!! MUST BE LAST CALL
        // if(has_nan(system)) cout << "NAN " << __LINE__ << endl;
    }

    if(rigid_only)
        energy::Energy::rigid_only(system);

    ///--- Solve & update parameter vector
    VectorN delta_thetas = energy::Energy::solve(system);
    const vector<float> dt(delta_thetas.data(), delta_thetas.data() + _thetas.size());
    _thetas = skeleton->getUpdatedParameters(_thetas, dt);

    ///--- Apply the transformation
    skeleton->set(_thetas);

    ///--- Update solution queue
    E_temporal.update(frame.id, _thetas);


    ///--- Return the tracking error
    // std::cout << "pull_error: " << pull_error << " push_error: " << push_error << std::endl;
    return TrackingError{pull_error, push_error};
}


 
