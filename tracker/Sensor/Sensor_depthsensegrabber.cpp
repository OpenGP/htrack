/*
Low pass filter from
http://www-users.cs.york.ac.uk/~fisher/cgi-bin/mkfscript
*/


#include "Sensor.h"
#include "util/mylogger.h"
#include "tracker/Data/DataFrame.h"

#ifndef HAS_SOFTKINETIC
SensorDepthSenseGrabber::SensorDepthSenseGrabber(Camera *camera) : Sensor(camera)
{
    LOG(FATAL) << "SoftKinetic not available in your OS";
}
int SensorDepthSenseGrabber::initialize(){ return 0; }
SensorDepthSenseGrabber::~SensorDepthSenseGrabber(){}
bool SensorDepthSenseGrabber::spin_wait_for_data(Scalar timeout_seconds){ return false; }
bool SensorDepthSenseGrabber::fetch_streams(DataFrame &frame){ return false; }
void SensorDepthSenseGrabber::start(){}
void SensorDepthSenseGrabber::stop(){}

#else

#include <DepthSense.hxx>
#include <stdio.h>
#include <vector>
#include <exception>
#include <thread>
#include <opencv2/opencv.hpp>
#include "util/Sleeper.h"
#include <iostream>
#include <limits>
#include <mutex>
#include <QElapsedTimer>
#include <QApplication>
#include <QEventLoop>
#include "tracker/Data/Camera.h"


using namespace DepthSense;
using namespace std;

namespace {
    const int BACK_BUFFER = 1;
    const int FRONT_BUFFER = 0;
}

int frameRateDepth = 60;
int frameRateColor = 30;

FrameFormat frameFormatDepth = FRAME_FORMAT_QVGA;
const int widthDepth = 320, heightDepth = 240;



/**********************************************************
 * Filter out inaccurate measurements with confidence map *
 * Fix finger doubling in color map                       *
 **********************************************************/

#define DEPTHSENSEGRABBER_USE_CONFIDENCE_MAP
#if defined(DEPTHSENSEGRABBER_USE_CONFIDENCE_MAP)
const uint16_t confidenceThreshold = 100;

#endif // DEPTHSENSEGRABBER_USE_CONFIDENCE_MAP


/*******************************************************
 * Spatial smoothing with Gaussian blur                *
 * Could be improved by only filtering up to the edges *
 *******************************************************/

#define DEPTHSENSEGRABBER_SMOOTH_SPATIAL

int kernel_length = 3;

/*******************************************
 * Temporal smoothing with low-pass filter *
 * http://www.exstrom.com/journal/sigproc/ *
 *******************************************/

#define DEPTHSENSEGRABBER_SMOOTH_TEMPORAL

uint16_t maxDeltaDepth = 50;

const int filterOrder = 2;
const int filterSize = filterOrder/2;
const int filterSamplingFreq = frameRateDepth;
const int filterCornerFreq = 15;

float filterA[filterSize];
float filterD1[filterSize];
float filterD2[filterSize];

float w0All[heightDepth*widthDepth*filterSize];
float w1All[heightDepth*widthDepth*filterSize];
float w2All[heightDepth*widthDepth*filterSize];

void initFilterWeights(float* A, float* d1, float* d2, int n, int s, int f) {
    float a = tan(M_PI*f/s);
    float a2 = a*a;
    for (int i = 0; i < n; i++) {
        float r = sin(M_PI*(2.0*i+1.0)/(4.0*n));
        float t = a2 + 2.0*a*r + 1.0;
        A[i] = a2/t;
        d1[i] = 2.0*(1-a2)/t;
        d2[i] = -(a2 - 2.0*a*r + 1.0)/t;
    }
}

uint16_t filterNew(uint16_t sample, float* w0, float* w1, float* w2, int n, float* A, float* d1, float* d2) {
    float x = static_cast<float>(sample);
    for(int i=0; i < n; ++i){
        w0[i] = d1[i]*w1[i] + d2[i]*w2[i] + x;
        x = A[i]*(w0[i] + 2.0*w1[i] + w2[i]);
        w2[i] = w1[i];
        w1[i] = w0[i];
    }
    uint16_t filteredVal = static_cast<uint16_t>(x);
    bool isInRange = abs(filteredVal - sample) < maxDeltaDepth;
    if (not isInRange) return sample;
    return static_cast<uint16_t>(x);
}



///--- @note we read VGA, but then sub-sample to QVGA
FrameFormat frameFormatColor = FRAME_FORMAT_VGA;
const int widthColor = 320, heightColor = 240;

uint8_t noDepthBGR[3] = {255,255,255};
int divideDepthBrightnessCV = 6;

Context g_context;
DepthNode g_dnode;
ColorNode g_cnode;

bool g_bDeviceFound = false;

ProjectionHelper* g_pProjHelper = NULL;
StereoCameraParameters g_scp;

#if 0
std::vector<cv::Mat> depth_raw;
std::vector<cv::Mat> depth_sync;
std::vector<cv::Mat> color_sync;
#endif

cv::Mat color_raw;
cv::Mat color[2];
cv::Mat depth[2];

std::pair<int, int> buffer_ids = std::make_pair(0,1);

int color_frames_count = 0;
int depth_frames_count = 0;

std::thread sensor_thread;
std::mutex swap_mutex;

void onNewColorSample(ColorNode, ColorNode::NewSampleReceivedData data);
void onNewDepthSample(DepthNode node, DepthNode::NewSampleReceivedData data);


void configureDepthNode() {
    g_dnode.newSampleReceivedEvent().connect(&onNewDepthSample);
    DepthNode::Configuration config = g_dnode.getConfiguration();
    config.frameFormat = frameFormatDepth;
    config.framerate = frameRateDepth;
    config.mode = DepthNode::CAMERA_MODE_CLOSE_MODE;
    config.saturation = true;

    g_dnode.setEnableDepthMap(true);
    g_dnode.setEnableUvMap(true);
#if defined(DEPTHSENSEGRABBER_USE_CONFIDENCE_MAP)
    g_dnode.setEnableConfidenceMap(true);
#endif // DEPTHSENSEGRABBER_USE_CONFIDENCE_MAP

    try {
        g_context.requestControl(g_dnode,0);
        g_dnode.setEnableDenoising(true); ///< why is it not working?
        g_dnode.setConfiguration(config);
    } catch (ArgumentException& e) {
        printf("DEPTH Argument Exception: %s\n",e.what());
    } catch (UnauthorizedAccessException& e) {
        printf("DEPTH Unauthorized Access Exception: %s\n",e.what());
    } catch (IOException& e) {
        printf("DEPTH IO Exception: %s\n",e.what());
    } catch (InvalidOperationException& e) {
        printf("DEPTH Invalid Operation Exception: %s\n",e.what());
    } catch (ConfigurationException& e) {
        printf("DEPTH Configuration Exception: %s\n",e.what());
    } catch (StreamingException& e) {
        printf("DEPTH Streaming Exception: %s\n",e.what());
    } catch (TimeoutException&) {
        printf("DEPTH TimeoutException\n");
    }
}


void configureColorNode() {
    // connect new color sample handler
    g_cnode.newSampleReceivedEvent().connect(&onNewColorSample);

    ColorNode::Configuration config = g_cnode.getConfiguration();
    config.frameFormat = frameFormatColor;
    config.compression = COMPRESSION_TYPE_MJPEG; // can also be COMPRESSION_TYPE_YUY2
    config.powerLineFrequency = POWER_LINE_FREQUENCY_50HZ;
    config.framerate = frameRateColor;

    g_cnode.setEnableColorMap(true);

    try {
        g_context.requestControl(g_cnode,0);
        g_cnode.setConfiguration(config);
        g_cnode.setBrightness(0);
        g_cnode.setContrast(5);
        g_cnode.setSaturation(5);
        g_cnode.setHue(0);
        g_cnode.setGamma(3);
        g_cnode.setWhiteBalance(4650);
        g_cnode.setSharpness(5);
        g_cnode.setWhiteBalanceAuto(true);
    } catch (ArgumentException& e) {
        printf("COLOR Argument Exception: %s\n",e.what());
    } catch (UnauthorizedAccessException& e) {
        printf("COLOR Unauthorized Access Exception: %s\n",e.what());
    } catch (IOException& e) {
        printf("COLOR IO Exception: %s\n",e.what());
    } catch (InvalidOperationException& e) {
        printf("COLOR Invalid Operation Exception: %s\n",e.what());
    } catch (ConfigurationException& e) {
        printf("COLOR Configuration Exception: %s\n",e.what());
    } catch (StreamingException& e) {
        printf("COLOR Streaming Exception: %s\n",e.what());
    } catch (TimeoutException&) {
        printf("COLOR TimeoutException\n");
    }
}


void configureNode(Node node) {
    if ((node.is<DepthNode>())&&(!g_dnode.isSet())) {
        g_dnode = node.as<DepthNode>();
        configureDepthNode();
        g_context.registerNode(node);
    }

    if ((node.is<ColorNode>())&&(!g_cnode.isSet())) {
        g_cnode = node.as<ColorNode>();
        configureColorNode();
        g_context.registerNode(node);
    }
}


void onNodeConnected(Device device, Device::NodeAddedData data) {
    configureNode(data.node);
}


void onNodeDisconnected(Device device, Device::NodeRemovedData data) {
    if (data.node.is<ColorNode>() && (data.node.as<ColorNode>() == g_cnode))
        g_cnode.unset();
    if (data.node.is<DepthNode>() && (data.node.as<DepthNode>() == g_dnode))
        g_dnode.unset();
    printf("Node disconnected\n");
}


void onDeviceConnected(Context context, Context::DeviceAddedData data) {
    if (!g_bDeviceFound) {
        data.device.nodeAddedEvent().connect(&onNodeConnected);
        data.device.nodeRemovedEvent().connect(&onNodeDisconnected);
        g_bDeviceFound = true;
    }
}


void onDeviceDisconnected(Context context, Context::DeviceRemovedData data) {
    g_bDeviceFound = false;
    printf("Device disconnected\n");
}


static void getFirstAvailableNode (Context context) {
    // obtain the list of devices attached to the host
    vector<Device> devices = context.getDevices();
    StereoCameraParameters stereo;

    if (devices.size() != 0) {
        for(int i = 0; i < devices.size(); i++) {
            stereo=devices[i].getStereoCameraParameters();
        }
    }

    ExtrinsicParameters extParams = stereo.extrinsics;
    IntrinsicParameters intDepthParams = stereo.depthIntrinsics;
    IntrinsicParameters intColorParams = stereo.colorIntrinsics;

    cout<<"extrinsic parameters"<<endl;
    cout<<"rotation matrix"<<endl;
    cout<<extParams.r11<<" "<<extParams.r12<<" "<<extParams.r13<<endl;
    cout<<extParams.r21<<" "<<extParams.r22<<" "<<extParams.r23<<endl;
    cout<<extParams.r31<<" "<<extParams.r32<<" "<<extParams.r33<<endl;
    cout<<endl;
    cout<<"translation vector"<<endl;
    cout<<extParams.t1<<" "<<extParams.t2<<" "<<extParams.t3<<endl;

    cout<<"the central point along the x and y axis, expressed in pixel units "<<endl;
    cout<<intDepthParams.cx<<" "<<intDepthParams.cy<<endl;
    cout<<"the focal length along the x and y axis, expressed in pixel units "<<endl;
    cout<<intDepthParams.fx<<" "<<intDepthParams.fy<<endl;
    cout<<"the height and width of the map when the frame was captured"<<endl;
    cout<<intDepthParams.height<<" "<<intDepthParams.width<<endl;
}


void run(){
    g_context.run();
}

void onNewColorSample(ColorNode, ColorNode::NewSampleReceivedData data) {
    // std::cout << "onNewColorSample " << color_frames_count << std::endl;
    color_frames_count++;

    for (int i=0, countColor=0; i<heightColor; i++) {
        for (int j=0; j<widthColor; j++) {
            int super_off = (i*2)*(2*widthColor) + (2*j);
            int r = data.colorMap[3*super_off+0];
            int g = data.colorMap[3*super_off+1];
            int b = data.colorMap[3*super_off+2];
            color_raw.at<cv::Vec3b>(cv::Point(j, i)) = cv::Vec3b(b, g, r);
            ///---
            countColor++;
        }
    }
}


void onNewDepthSample(DepthNode node, DepthNode::NewSampleReceivedData data) {
    // std::cout << "onNewDepthSample " << depth_frames_count << std::endl;

    int countDepth = 0;
    for (int i=0; i < heightDepth; i++) {
        for (int j=0; j < widthDepth; j++) {
            int d = data.depthMap[countDepth];
#if 0
            if (d >= camera->zFar())
                d = camera->zFar();
            if (d <= camera->zNear())
                d = camera->zFar();
#endif
           //cout << d << " ";


#if defined(DEPTHSENSEGRABBER_SMOOTH_TEMPORAL)
            float* w0 = w0All + countDepth*filterSize;
            float* w1 = w1All + countDepth*filterSize;
            float* w2 = w2All + countDepth*filterSize;
            d = filterNew(d, w0, w1, w2, filterSize, filterA, filterD1, filterD2);
#endif // DEPTHSENSEGRABBER_SMOOTH_TEMPORAL

            ///--- Fetch color using UV map + raw color
            UV uv = data.uvMap[countDepth];
            int colorPixelRow = round(uv.v * heightColor); ///< -1??
            int colorPixelCol = round(uv.u * widthColor); ///< -1??
            bool in_range = (colorPixelCol < widthColor && colorPixelCol >= 0 && colorPixelRow < heightColor && colorPixelRow >= 0);
#if defined(DEPTHSENSEGRABBER_USE_CONFIDENCE_MAP)
            uint16_t confidenceVal = data.confidenceMap[countDepth];
            in_range *= (confidenceVal > confidenceThreshold);
#endif // DEPTHSENSEGRABBER_USE_CONFIDENCE_MAP
            cv::Vec3b col = in_range ? color_raw.at<cv::Vec3b>(cv::Point(colorPixelCol, colorPixelRow)) : cv::Vec3b(255,255,255);


            ///--- Set buffers
            int j_mirror = widthDepth -j -1;
            //depth[BACK_BUFFER].at<unsigned short>(cv::Point(j_mirror, i)) = d;
            depth[BACK_BUFFER].at<uint16_t>(cv::Point(j_mirror, i)) = d;
            color[BACK_BUFFER].at<cv::Vec3b>(cv::Point(j_mirror, i)) = col;

            ///---
            countDepth++;
        }
    }

#if defined(DEPTHSENSEGRABBER_SMOOTH_SPATIAL)
    cv::GaussianBlur( depth[BACK_BUFFER], depth[BACK_BUFFER], cv::Size( kernel_length, kernel_length ), 0, 0 );
#endif // DEPTHSENSEGRABBER_SMOOTH_SPATIAL

    ///--- safe swap front & back
    // std::lock_guard<std::mutex> lock(color_mutex);
    while (true) {
        if (swap_mutex.try_lock()) {
            std::swap( depth[0], depth[1] ); ///< because operator= is shallow
            std::swap( color[0], color[1] ); ///< because operator= is shallow
            depth_frames_count++;
            swap_mutex.unlock();
            break;
        }
    }
}


SensorDepthSenseGrabber::SensorDepthSenseGrabber(Camera *camera) : Sensor(camera){
    if(camera->mode() != Intel)
        LOG(FATAL) << "SoftKinetic needs Intel camera mode";
}

int SensorDepthSenseGrabber::initialize(){
    std::cout << "SensorDepthSenseGrabber::initialize()" << std::endl;

    g_context = Context::create("localhost");

    g_context.deviceAddedEvent().connect(&onDeviceConnected);
    g_context.deviceRemovedEvent().connect(&onDeviceDisconnected);

    // Get the list of currently connected devices
    vector<Device> da = g_context.getDevices();

    // We are only interested in the first device
    if (da.size() >= 1)
    {
        g_bDeviceFound = true;

        da[0].nodeAddedEvent().connect(&onNodeConnected);
        da[0].nodeRemovedEvent().connect(&onNodeDisconnected);

        vector<Node> na = da[0].getNodes();

        printf("Found %lu nodes\n",na.size());

        for (int n = 0; n < (int)na.size(); n++)
            configureNode(na[n]);
    }

#if 0
    depth_raw.resize(2);
    color_raw.resize(2);
    depth_sync.resize(2);
    color_sync.resize(2);
#else
    color_raw = cv::Mat(heightColor, widthColor, CV_8UC3, cv::Scalar(0,0,0));
    ///--- Buffers
    color[FRONT_BUFFER] = cv::Mat(heightColor, widthColor, CV_8UC3, cv::Scalar(0,0,0));
    color[BACK_BUFFER]  = cv::Mat(heightColor, widthColor, CV_8UC3, cv::Scalar(0,0,0));
    depth[FRONT_BUFFER] = cv::Mat(heightDepth, widthDepth, CV_16UC1, cv::Scalar(0));
    depth[BACK_BUFFER] = cv::Mat(heightDepth, widthDepth, CV_16UC1, cv::Scalar(0));
#endif

    g_context.startNodes();
    sensor_thread = std::thread(run);
    this->initialized = true;
    return true;
}

SensorDepthSenseGrabber::~SensorDepthSenseGrabber(){
    std::cout << "~SensorDepthSenseGrabber()" << std::endl;

    ///--- All resources was done in initialize()
    if(!initialized) return;

    /// stop sensor
    g_context.stopNodes();
    if (g_cnode.isSet()) g_context.unregisterNode(g_cnode);
    if (g_dnode.isSet()) g_context.unregisterNode(g_dnode);
    if (g_pProjHelper)
        delete g_pProjHelper;

    /// kill thread
    // sensor_thread.join();
}


bool SensorDepthSenseGrabber::spin_wait_for_data(Scalar timeout_seconds){
    if( (depth_frames_count>0) && (color_frames_count>0) )
        return true;

    QElapsedTimer chrono;
    chrono.start();

    while( (depth_frames_count==0) || (color_frames_count==0) ) {
        LOG(INFO) << "Waiting for data... " << chrono.elapsed();
        Sleeper::msleep(500);
        QApplication::processEvents(QEventLoop::AllEvents);
        if( chrono.elapsed() > 1000*timeout_seconds )
            return false;
    }


    return true;
}

bool SensorDepthSenseGrabber::fetch_streams(DataFrame &frame){

#if 0
    frame.color = color_sync[back];
    frame.depth = depth_raw[back];
#else
    // frame.depth = fake_depth.clone();
    while (true) {
        if (swap_mutex.try_lock()) {
            frame.color = color[FRONT_BUFFER].clone();
            frame.depth = depth[FRONT_BUFFER].clone();
            //std::cout << "color = " << color_frames_count << endl;
            //std::cout << "depth = " << depth_frames_count << endl;
            swap_mutex.unlock();
            break;
        }
    }
#endif



    /*unsigned short mval = std::numeric_limits<unsigned short>::max();
    cv::namedWindow( "Color raw",cv::WINDOW_AUTOSIZE );
    cv::imshow( "Color raw", color_raw[back]);

    cv::normalize(depth_sync[back], depth_sync[back], 0, mval, cv::NORM_MINMAX);
    cv::namedWindow( "Depth sync",cv::WINDOW_AUTOSIZE );
    cv::imshow( "Depth sync", depth_sync[back]);

    cv::normalize(depth_raw[back], depth_raw[back], 0, mval, cv::NORM_MINMAX);
    cv::namedWindow( "Depth raw",cv::WINDOW_AUTOSIZE );
    cv::imshow( "Depth raw", depth_raw[back]);

    cv::namedWindow( "Color sync",cv::WINDOW_AUTOSIZE );
    cv::imshow( "Color sync", color_sync[back]);

    cvWaitKey(5);*/

    return true;
}

void SensorDepthSenseGrabber::start()
{
#if defined (DEPTHSENSEGRABBER_SMOOTH_TEMPORAL)
    initFilterWeights(filterA, filterD1, filterD2, filterSize, filterSamplingFreq, filterCornerFreq);
#endif //DEPTHSENSEGRABBER_SMOOTH_TEMPORAL
    if(!initialized)
        this->initialize();
    g_context.startNodes();
}

void SensorDepthSenseGrabber::stop()
{
    g_context.stopNodes();
}

#endif





