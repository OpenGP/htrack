#include <iostream>
#include "pxcsensemanager.h"
#include "pxcsession.h"
#include "pxcprojection.h"

#include "util/tictoc.h"
#include "util/opencv_wrapper.h"
#include "tracker/Sensor/Sensor.h"
#include "tracker/Data/DataFrame.h"

int main(int /*argc*/, char ** /*argv*/){
    Camera camera(QVGA, 60 /*FPS*/);
    SensorRealSense sensor(&camera);

    ///--- start the sensor
    sensor.start();
    bool success = sensor.spin_wait_for_data(5 /*seconds*/);
    assert(success);

    DataFrame frame(0);
    for(int i=0; i<30 /*seconds*/ *60 /*60FPS*/; i++){
        tic(total);
        bool success = sensor.fetch_streams(frame);
        assert(success);
        // printf("fetch time: %2.2f\n", toc(total)); fflush(stdout);
        // cv::normalize(frame.color, frame.color, 0, 255, cv::NORM_MINMAX);

        ///--- Show color
        cvtColor(frame.color, frame.color, CV_BGR2RGB);
        cv::imshow("color", frame.color);
        ///--- Show depth
        cv::normalize(frame.depth, frame.depth, 0, USHRT_MAX, cv::NORM_MINMAX);
        cv::imshow("depth", frame.depth);

        ///--- Wait a sec
        cv::waitKey(15 /*ms -> 60fps*/);
    }
    return 0;
}

#if 0
/// @see file:///C:/Developer/RealSense/RSSDK/doc/HTML/index.html
class MyHandler : public PXCSenseManager::Handler {
public:
    virtual pxcStatus PXCAPI OnNewSample(pxcUID, PXCCapture::Sample *sample) {
        static auto now = std::chrono::system_clock::now();

        // Work on sample->color and sample->depth
        // ...
        // return NO_ERROR to continue, or any error to exit the loop
        if (sample->color!=NULL) {
            // work on the color sample
        }
        if (sample->depth!=NULL) {
            // work on the depth sample (actual useful data)
            auto now2 = std::chrono::system_clock::now();
            double elapsed = std::chrono::duration <double, milli> (now2-now).count();
            printf("elapsed: %.2f", elapsed); fflush(stdout);
            now = now2;
        }

        return PXC_STATUS_NO_ERROR;
    }
};

int main(int /*argc*/, char ** /*argv*/){
   // Create a SenseManager instance
   PXCSenseManager *sm=PXCSenseManager::CreateInstance();

#define SYNCHRONIZED
#ifdef SYNCHRONIZED
   /// C++ Example 26: Capture Aligned Color and Depth Samples using the SenseManager Events
   // Select the color and depth streams
   PXCVideoModule::DataDesc ddesc={};
   ddesc.deviceInfo.streams=PXCCapture::STREAM_TYPE_COLOR|PXCCapture::STREAM_TYPE_DEPTH;
   ddesc.streams.color.frameRate = {60.0f, 60.0f};
   ddesc.streams.color.sizeMin = {320, 240};
   ddesc.streams.color.sizeMax = {320, 240};
   ddesc.streams.depth.frameRate = {60.0f, 60.0f};
   ddesc.streams.depth.sizeMin = {640, 240};
   ddesc.streams.depth.sizeMax = {640, 240};
   sm->EnableStreams(&ddesc);
#else
   /// C++ Example 25: Capture Unaligned Color and Depth Samples using the SenseManager Events
   sm->EnableStream(PXCCapture::STREAM_TYPE_COLOR,320,240,60);
   sm->EnableStream(PXCCapture::STREAM_TYPE_DEPTH,640,240,60);
#endif

   // Initialize my handler
   MyHandler handler;
   sm->Init(&handler);

   // Stream Data
   sm->StreamFrames(true);

   // Clean up
   sm->Release();
   return 0;
}
#endif

#if 0
int main(int /*argc*/, char ** /*argv*/){
    PXCSession* session = PXCSession::CreateInstance();
    PXCSession::ImplDesc desc={};
    desc.group=PXCSession::IMPL_GROUP_SENSOR;
    desc.subgroup=PXCSession::IMPL_SUBGROUP_VIDEO_CAPTURE;
    for (int i=0;;i++) {
       PXCSession::ImplDesc desc1;
       if (session->QueryImpl(&desc,i,&desc1)<PXC_STATUS_NO_ERROR) break;

       ///--- Create capture device
       PXCCapture *capture;
       session->CreateImpl<PXCCapture>(&desc1,&capture);

       for (int i=0;;i++) {
          PXCCapture::DeviceInfo dinfo;
          if (capture->QueryDeviceInfo(i,&dinfo)<PXC_STATUS_NO_ERROR) break;
          wprintf_s(L"device[%d]: %s\n", i, dinfo.name);
       }

       PXCCapture::Device* device = capture->CreateDevice(0);
       PXCCapture::StreamType streams=PXCCapture::STREAM_TYPE_COLOR|PXCCapture::STREAM_TYPE_DEPTH;
       for (int p=0;;p++) {
          PXCCapture::Device::StreamProfileSet profiles={};
          pxcStatus sts=device->QueryStreamProfileSet(streams, p, &profiles);
          if (sts<PXC_STATUS_NO_ERROR) break;

          if(profiles.depth.imageInfo.width < 641 )
            printf("[%d] color: %dx%d@[%d %d]  depth: %dx%d@[%d %d]\n", p,
                   profiles.color.imageInfo.width,
                   profiles.color.imageInfo.height,
                   profiles.color.frameRate.min, profiles.color.frameRate.max,
                   profiles.depth.imageInfo.width,
                   profiles.depth.imageInfo.height,
                   profiles.depth.frameRate.min, profiles.color.frameRate.max);
       }

       device->Release();
       capture->Release();
    }
    return 0;
}
#endif

#if 0
int main(int /*argc*/, char ** /*argv*/){
    PXCSenseManager *sm=PXCSenseManager::CreateInstance();

    // Select the color and depth streams
    sm->EnableStream(PXCCapture::STREAM_TYPE_COLOR,320,240,60);
    sm->EnableStream(PXCCapture::STREAM_TYPE_DEPTH,640,240,60);

    // Initialize and Stream Samples
    sm->Init();
    for (int i=0;i<20;i++) {
        // This function blocks until both samples are ready
        if (sm->AcquireFrame(true)<PXC_STATUS_NO_ERROR){
           std::cout << "AcquireFrame Error" << std::endl;
           break;
        }
        std::cout << "RUNNING: " << i << std::endl;

        // retrieve the color and depth samples aligned
        PXCCapture::Sample *sample=sm->QuerySample();

        // work on the samples sample->color and sample->depth

        // go fetching the next samples
        sm->ReleaseFrame();
    }

    // Close down
    sm->Release();
    exit(0);
    std::cout << "hello world!" << std::endl;
    return 0;
}
#endif
