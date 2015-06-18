#include "util/gl_wrapper.h"

#include <QApplication>
#include <QString>
#include <QDebug>
#include <QDir>
#include <QThread>


#include "OpenGL_viewer.h"
#include "Main_window.h"
#include "util/mylogger.h"

#include "tracker/ForwardDeclarations.h"
#include "tracker/Sensor/Sensor.h"
#include "tracker/Tracker.h"
#include "tracker/Worker.h"
#include "tracker/Data/DataStream.h"
#include "tracker/Data/SolutionStream.h"
#include "tracker/DataStructure/SkeletonSerializer.h"
#include "tracker/Calibration/Calibration.h"

int main(int argc, char* argv[]) {
	Q_INIT_RESOURCE(shaders); ///< http://qt-project.org/wiki/QtResources

    LOG(INFO) << "htrack starting";
    LOG(INFO) << "--Execution path: " << QDir::currentPath();
    QString filename_data = (argc >= 2) ? argv[1] : "";
    QString filename_soln = (argc >= 3) ? argv[2] : "";

    ///--- Setup application
    QApplication app(argc, argv);
    QThread::currentThread()->setObjectName("Main thread");
    QThread::currentThread()->setPriority(QThread::HighestPriority);

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
    datastream.load(filename_data);

    SolutionStream* solutions = new SolutionStream();
    Worker worker(&camera);
    OpenGL_viewer* glarea = new OpenGL_viewer(&worker);
    worker.bind_glarea(glarea);

    Main_window* window = new Main_window(&worker, &datastream, solutions, glarea);
    window->raise();
    window->show();

    Tracker tracker(&worker, camera.FPS());
    tracker.sensor = &sensor;
    tracker.datastream = &datastream;
    tracker.solutions = solutions;

    ///--- UI connections
    QObject::connect(window->toggle_record_, &QPushButton::toggled, &tracker, &Tracker::toggle_recording);
    QObject::connect(window->toggle_live_, &QPushButton::toggled, &tracker, &Tracker::toggle_tracking);

    ///--- Attempt to load data
    solutions->resize(datastream.size());

    int debug_init_with_frame = 0; ///< decide where to start
    if (!filename_soln.isEmpty())
        solutions->load(filename_soln);
    if (datastream.size() > 0) {
        window->display_frame(debug_init_with_frame);
    }

    ///--- Load calibration
    Calibration(&worker).autoload();
    glarea->reload_model();

    ///--- Flip the hand model if desired
//#define RIGHT_HAND
#ifdef RIGHT_HAND
    /// special case for left-handed calibration...
    if (!datastream.get_prefix().startsWith("maschroemelax")) {
        // flip mapping and model
        worker.skeleton->toRightHand2();
    }
    else {
        // flip mapping only and update
        worker.skeleton->setMapping(Mapping::rightArm2());
        if (datastream.size() > 0) {
            window->display_frame(debug_init_with_frame);
        }
    }
#endif

    ///--- Start event loop
    return app.exec();
}
