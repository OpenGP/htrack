#include "Sensor.h"
#include "tracker/Types.h"

#include "util/gl_wrapper.h"
#include "util/mylogger.h"
#include "util/Sleeper.h"
#include "util/tictoc.h"


#ifndef HAS_LIBREALSENSE
    SensorLibRealSense::SensorLibRealSense(Camera *camera) : Sensor(camera){ mFatal() << "Intel LibRealSense not available in your OS"; }
    int SensorLibRealSense::initialize(){ return 0; }
    SensorLibRealSense::~SensorLibRealSense(){}
    bool SensorLibRealSense::spin_wait_for_data(Scalar timeout_seconds){ return false; }
    bool SensorLibRealSense::fetch_streams(DataFrame &frame){ return false; }
    void SensorLibRealSense::start(){}
    void SensorLibRealSense::stop(){}
#else

#include "Sensor.h"

#include <stdio.h>
#include <vector>
#include <exception>
#include <opencv2/opencv.hpp>

#include <iostream>
#include <limits>
#include <QElapsedTimer>
#include <QApplication>
#include <QMessageBox>
#include <librealsense/rs.hpp>
#include "tracker/Data/DataFrame.h"
#include "tracker/Data/Camera.h"

using namespace std;


rs::context ctx;
rs::device * dev;
rs::intrinsics depth_intrin, color_intrin;
rs::extrinsics depth_to_color;

int D_width  = 640;
int D_height = 480;
float scale;

SensorLibRealSense::SensorLibRealSense(Camera *camera) : Sensor(camera) {
    if (camera->mode() != Intel)
        LOG(FATAL) << "!!!FATAL: LibRealSense needs Intel camera mode";
}

int SensorLibRealSense::initialize() {
    std::cout << "SensorLibRealSense::initialize()" << std::endl;


    if (ctx.get_device_count() == 0) return EXIT_FAILURE;

    dev = ctx.get_device(0);
    printf("\nUsing device 0, an %s\n", dev->get_name());
    printf("    Serial number: %s\n", dev->get_serial());
    printf("    Firmware version: %s\n", dev->get_firmware_version());

    dev->enable_stream(rs::stream::depth,  D_width, D_height, rs::format::z16, 60);
    dev->enable_stream(rs::stream::color, D_width, D_height, rs::format::rgb8, 60);

    depth_intrin = dev->get_stream_intrinsics(rs::stream::depth);
    depth_to_color = dev->get_extrinsics(rs::stream::depth, rs::stream::color);
    color_intrin = dev->get_stream_intrinsics(rs::stream::color);
    scale = dev->get_depth_scale();

    printf("Enabled Streams:Depth and Color\n");

    dev->start();
    printf("Device Started\n");

    this->initialized = true;
    return true;
}

SensorLibRealSense::~SensorLibRealSense() {
    std::cout << "~SensorLibRealSense()" << std::endl;

    if (!initialized) return;
    // TODO: stop sensor

}

bool SensorLibRealSense::spin_wait_for_data(Scalar timeout_seconds) {

    DataFrame frame(-1);
    QElapsedTimer chrono;
    chrono.start();
    while (fetch_streams(frame) == false) {
        LOG(INFO) << "Waiting for data.. " << chrono.elapsed();
        Sleeper::msleep(500);
        QApplication::processEvents(QEventLoop::AllEvents);
        if (chrono.elapsed() > 1000 * timeout_seconds)
            return false;
    }
    return true;
}

bool SensorLibRealSense::fetch_streams(DataFrame &frame) {
    if (initialized == false) this->initialize();

    if(frame.depth.empty())
        frame.depth = cv::Mat(cv::Size(D_width/2, D_height/2), CV_16UC1, cv::Scalar(0));
    if(frame.color.empty())
        frame.color = cv::Mat(cv::Size(D_width/2, D_height/2), CV_8UC3, cv::Scalar(0, 0, 0));

    dev->poll_for_frames();
    const uint16_t * depth_image = (const uint16_t *)dev->get_frame_data(rs::stream::depth);
    const uint8_t * color_image = (const uint8_t *)dev->get_frame_data(rs::stream::color);



    cv::Mat depth_buffer = cv::Mat(cv::Size(D_width/2, D_height/2), CV_16UC1, cv::Scalar(0));
    cv::Mat color_buffer = cv::Mat(cv::Size(D_width/2, D_height/2), CV_8UC3, cv::Scalar(255,255,255));
    for(int dy=0,dy_sub=0; dy<depth_intrin.height; dy+=2,dy_sub++)
    {
        for(int dx=0,dx_sub=0; dx<depth_intrin.width; dx+=2,dx_sub++)
        {
            uint16_t depth_value = depth_image[dy * depth_intrin.width + (depth_intrin.width-dx-1)];
            float depth_in_meters = depth_value * scale;
            float pixel_depth_in_mm = depth_in_meters * 1000;
            if(depth_value == 0) continue;

            rs::float2 depth_pixel = {(float)(depth_intrin.width-dx-1), (float)dy};
            rs::float3 depth_point = depth_intrin.deproject(depth_pixel, depth_in_meters);
            rs::float3 color_point = depth_to_color.transform(depth_point);
            rs::float2 color_pixel = color_intrin.project(color_point);
            const int cx = (int)std::round(color_pixel.x), cy = (int)std::round(color_pixel.y);
            if(cx < 0 || cy < 0 || cx >= color_intrin.width || cy >= color_intrin.height)
            {
                color_buffer.at<cv::Vec3b>(dy_sub, dx_sub) = cv::Vec3b(255,255,255);
            }
            else
            {
                unsigned char r = color_image[cy * D_width * 3 + (cx) * 3 + 0];
                unsigned char g = color_image[cy * D_width * 3 + (cx) * 3 + 1];
                unsigned char b = color_image[cy * D_width * 3 + (cx) * 3 + 2];
                color_buffer.at<cv::Vec3b>(dy_sub, dx_sub) = cv::Vec3b(r,g,b);
                depth_buffer.at<unsigned short>(dy_sub,dx_sub) = (unsigned short)pixel_depth_in_mm;
            }
        }
    }
    frame.color = color_buffer;
    frame.depth = depth_buffer;
    return true;
}

void SensorLibRealSense::start() {
    if (!initialized)
        this->initialize();
}

void SensorLibRealSense::stop() {
}
#endif
