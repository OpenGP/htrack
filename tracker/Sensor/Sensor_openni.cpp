#include "Sensor.h"
#include "util/mylogger.h"
#include "util/Sleeper.h"

#include "tracker/Types.h"
#include "tracker/Data/DataFrame.h"
#include "tracker/Data/Camera.h"

#ifndef WITH_OPENNI
void openni_hard_quit(){ std::cout << "!!!ERROR: Cannot use SensorOpenNI" << std::endl; }
SensorOpenNI::SensorOpenNI(Camera* camera) : Sensor(camera) { openni_hard_quit(); }
SensorOpenNI::~SensorOpenNI(){ openni_hard_quit(); }
bool SensorOpenNI::spin_wait_for_data(Scalar timeout_seconds){ openni_hard_quit(); return false; }
bool SensorOpenNI::fetch_streams(DataFrame& frame){ openni_hard_quit(); return false; }
int SensorOpenNI::initialize(){ openni_hard_quit(); return 0; }
#else
#include "OpenNI.h"
#include <QObject>
#include <QElapsedTimer>
#include <QEventLoop>
#include <QApplication>

#ifdef __APPLE__
    namespace kinect = openni;
#endif
#ifdef _WIN32
    namespace kinect = openni;
#endif

/// Device
kinect::Device device;

/// Streams
kinect::VideoStream g_depthStream;
kinect::VideoStream g_colorStream;

/// Frames
kinect::VideoFrameRef g_depthFrame;
kinect::VideoFrameRef g_colorFrame;

SensorOpenNI::SensorOpenNI(Camera *camera) : Sensor(camera) {
    if(camera->mode() != QVGA)
        LOG(FATAL) << "OpenNI sensor needs QVGA camera mode";
}

int SensorOpenNI::initialize()
{
    LOG(INFO) << "Initializing OpenNI";
    ///< force shutdown before starting!!
    kinect::OpenNI::shutdown();

    kinect::Status rc;
    rc = kinect::STATUS_OK;

    /// Fetch the device URI to pass to Device::open()
    const char* deviceURI = kinect::ANY_DEVICE;

    /// Initialize the device
    rc = kinect::OpenNI::initialize();
    if(rc!=kinect::STATUS_OK)
    {
        mDebug()<<"Initialization Errors (if any): "<< kinect::OpenNI::getExtendedError();
        kinect::OpenNI::shutdown();
        exit(0);
    }

    /// Open the device using the previously fetched device URI
    rc = device.open(deviceURI);
    if (rc != kinect::STATUS_OK)
    {
        mDebug()<<"Device open failed: "<<kinect::OpenNI::getExtendedError();
        kinect::OpenNI::shutdown();
        exit(0);
    }

    /// Create the depth stream
    rc = g_depthStream.create(device, kinect::SENSOR_DEPTH);
    if (rc == kinect::STATUS_OK)
    {
        /// start the depth stream, if its creation was successful
        rc = g_depthStream.start();

        if (rc != kinect::STATUS_OK)
        {
            mDebug()<<"Couldn't start depth stream: "<<kinect::OpenNI::getExtendedError();
            g_depthStream.destroy();
            exit(0);
        }
    }
    else
    {
        mDebug()<<"Couldn't find depth stream: "<<kinect::OpenNI::getExtendedError();
        exit(0);
    }

    if (!g_depthStream.isValid())
    {
        mDebug()<<"No valid depth streams. Exiting";
        kinect::OpenNI::shutdown();
        exit(0);
    }

    /// Create the color stream
    rc = g_colorStream.create(device, kinect::SENSOR_COLOR);

    if (rc == kinect::STATUS_OK)
    {
        /// start the color stream, if its creation was successful
        rc = g_colorStream.start();

        if (rc != kinect::STATUS_OK)
        {
            mDebug()<<"Couldn't start color stream: "<<kinect::OpenNI::getExtendedError();
            g_colorStream.destroy();
            exit(0);
        }
    }
    else
    {
        mDebug()<<"Couldn't find color stream: "<<kinect::OpenNI::getExtendedError();
        exit(0);
    }

    if (!g_colorStream.isValid())
    {
        mDebug()<<"No valid color streams. Exiting";
        kinect::OpenNI::shutdown();
        exit(0);
    }

    /// Configure resolutions
    {
        /// Attempt to set for depth
        {
            kinect::VideoMode mode = g_depthStream.getVideoMode();
            if(((int)camera->FPS())==60)
                mode.setFps(60);
            else
                mode.setFps(30);
            mode.setResolution(camera->width(), camera->height());
            rc = g_depthStream.setVideoMode(mode);
            if (rc != kinect::STATUS_OK)
                std::cerr << "error setting video mode (depth)" << std::endl;
        }
        /// Attempt to set for color
        {
            kinect::VideoMode mode = g_colorStream.getVideoMode();
            if(((int)camera->FPS())==60)
                mode.setFps(60);
            else
                mode.setFps(30);
            mode.setFps(30); ///< @todo check!!!
            mode.setResolution(camera->width(), camera->height());
            rc = g_colorStream.setVideoMode(mode);
            if (rc != kinect::STATUS_OK)
                std::cerr << "error setting video mode (color)" << std::endl;
        }
    }


#ifdef THIS_CAUSES_INIT_STALLS
    /// Enable depth/color frame synchronization
    rc = device.setDepthColorSyncEnabled(true);
    if (rc != kinect::STATUS_OK)
    {
        qDebug()<<"Could not synchronise device";
        // VGA Kinect always seems to shut down here
        kinect::OpenNI::shutdown();
        exit(0);
    }
#endif

    /// Camera settings
    kinect::CameraSettings* settings = g_colorStream.getCameraSettings();
    settings->setAutoExposureEnabled(true);
    settings->setAutoWhiteBalanceEnabled(true);

    /// Fetch the camera intrinsics
#if 0
        float w = g_depthStream.getVideoMode().getResolutionX();protected:
        Camera*const camera;
        /// Device
        kinect::Device device;
        bool initialized;

        /// Streams
        kinect::VideoStream g_depthStream;
        kinect::VideoStream g_colorStream;

        /// Frames
        kinect::VideoFrameRef g_depthFrame;
        kinect::VideoFrameRef g_colorFrame;
        float fov_h = g_depthStream.getHorizontalFieldOfView();
        float fl_h = .5*w / tan(.5*fov_h);
        float h = g_depthStream.getVideoMode().getResolutionY();
        float fov_v = g_depthStream.getVerticalFieldOfView();
        float fl_v = .5*h / tan(.5*fov_v);
        std::cout << "cameras focal lengths" << fl_h << fl_v;
#endif

    initialized = true;
    return true;
}

SensorOpenNI::~SensorOpenNI()
{
    if(initialized){
        LOG(INFO) << "Shutting down Kinect...";
        flush(std::cout);
        g_depthStream.destroy();
        g_colorStream.destroy();
        device.close();
        kinect::OpenNI::shutdown();
    }
}

bool SensorOpenNI::spin_wait_for_data(Scalar timeout_seconds)
{
    DataFrame frame(-1);
    QElapsedTimer chrono;
    chrono.start();
    while(fetch_streams(frame)==false){
        LOG(INFO) << "Waiting for data.. " << chrono.elapsed();
        Sleeper::msleep(500);
        QApplication::processEvents(QEventLoop::AllEvents);
        if( chrono.elapsed() > 1000*timeout_seconds )
            return false;
    }

    return true;
}

bool SensorOpenNI::fetch_streams(DataFrame &frame)
{
    if(initialized==false)
        this->initialize();

    /// @note WE DO IT HERE OTHERWISE IT GETS IGNORED IF WE DO IT IN INITIALIZATION :(
    /// Depth image is transformed to have the same apparent vantage point as the RGB image
    kinect::Status m_rc = device.setImageRegistrationMode(kinect::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
    if (m_rc != kinect::STATUS_OK)
    {
        mDebug()<<"Could not set Image Registration Mode";
        kinect::OpenNI::shutdown();
        exit(0);
    }

    kinect::VideoStream* streams[] = {&g_depthStream, &g_colorStream};
    int changedIndex = -1;
    kinect::Status rc = kinect::STATUS_OK;
    while (rc == kinect::STATUS_OK){
        rc = kinect::OpenNI::waitForAnyStream(streams, 2, &changedIndex, 0);

        if (rc == kinect::STATUS_OK){
            switch (changedIndex){
            case 0:
                // timer.restart();
                g_depthStream.readFrame(&g_depthFrame);
                // qDebug() << "depth.readFrame" << timer.restart();
                break;
            case 1:
                g_colorStream.readFrame(&g_colorFrame);
                // qDebug() << "color.readFrame" << timer.restart();
                break;
            default:
                printf("Error in wait\n");
            }
        }
    }

    // qDebug() << "\n Frames: ";
    // if(g_colorFrame.isValid())   qDebug("color: %d",g_colorFrame.getFrameIndex());
    // if(g_depthFrame.isValid())   qDebug("depth: %d",g_depthFrame.getFrameIndex());

    if( !g_colorFrame.isValid() ) return false;
    if( !g_depthFrame.isValid() ) return false;

    //--- DEBUG: identify bottleneck
    // qDebug() << "D - frameID: " << g_depthFrame.getFrameIndex();
    // qDebug() << "C - frameID: " << g_colorFrame.getFrameIndex();

    /// @note this DOES NOT copy memory!
    const kinect::RGB888Pixel* color_buffer = (const kinect::RGB888Pixel*) g_colorFrame.getData();
    const kinect::DepthPixel*  depth_buffer = (const kinect::DepthPixel*) g_depthFrame.getData();
    frame.color = cv::Mat(camera->height(), camera->width(), CV_8UC3, (void*) color_buffer);
    frame.depth = cv::Mat(camera->height(), camera->width(), CV_16UC1, (void*) depth_buffer);
    // qDebug() << "*stream->add_frame()" << timer.restart();
    return true;
}

#endif


#ifdef DOES_NOT_WORK
public slots:
    void force_reinit(){
        cout << "====> FORCING DEVICE RE-INITIALIZATION" << endl;
        this->shutdown();
        this->initialized = false;
        Sleeper::msleep(500/*ms*/); //< give it time... sigh
        hidden::kinect::OpenNI::initialize();
        cout << "====> (DONE) FORCING DEVICE RE-INITIALIZATION" << endl;
    }
#endif

#ifdef DISABLED
public:
    void auto_white_balance(bool flag){
        openni::CameraSettings* settings = g_colorStream.getCameraSettings();
        settings.setAutoWhiteBalanceEnabled(flag);
    }
    void auto_exposure(bool flag){
        openni::CameraSettings* settings = g_colorStream.getCameraSettings();
        settings.setAutoExposureEnabled(flag);
    }

public:
    void force_reinit(){
        initialized = false;
        openni::OpenNI::shutdown();
        initialize();
    }
#endif
