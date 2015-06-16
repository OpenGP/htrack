#include "Sensor.h"
#include "tracker/Types.h"

#include "util/gl_wrapper.h"
#include "util/mylogger.h"
#include "util/Sleeper.h"
#include "util/tictoc.h"


#ifndef HAS_REALSENSE
    SensorRealSense::SensorRealSense(Camera *camera) : Sensor(camera){ mFatal() << "Intel RealSense not available in your OS"; }
    int SensorRealSense::initialize(){ return 0; }
    SensorRealSense::~SensorRealSense(){}
    bool SensorRealSense::spin_wait_for_data(Scalar timeout_seconds){ return false; }
    bool SensorRealSense::fetch_streams(DataFrame &frame){ return false; }
    void SensorRealSense::start(){}
    void SensorRealSense::stop(){}

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

#include "pxcsensemanager.h"
#include "pxcsession.h"
#include "pxcprojection.h"
#include "util_render.h"  
#include <opencv2/highgui/highgui.hpp>
#include "tracker/Data/DataFrame.h"

using namespace std;

PXCImage::ImageData depth_buffer;
PXCImage::ImageData color_buffer;

PXCSenseManager *sense_manager;
PXCProjection *projection;

int D_width  = 640;
int D_height = 480;

SensorRealSense::SensorRealSense(Camera *camera) : Sensor(camera) {
	if (camera->mode() != Intel)
        LOG(FATAL) << "!!!FATAL: RealSense needs Intel camera mode";
}

int SensorRealSense::initialize() {
	std::cout << "SensorRealSense::initialize()" << std::endl;
	sense_manager = PXCSenseManager::CreateInstance();
	if (!sense_manager) {
		wprintf_s(L"Unable to create the PXCSenseManager\n");
		return false;
	}
    sense_manager->EnableStream(PXCCapture::STREAM_TYPE_COLOR, D_width/2, D_height/2, 60);
    sense_manager->EnableStream(PXCCapture::STREAM_TYPE_DEPTH, D_width, D_height, 60);
	sense_manager->Init();

	PXCSession *session = PXCSession::CreateInstance();
	PXCSession::ImplDesc desc, desc1;
	memset(&desc, 0, sizeof(desc));
	desc.group = PXCSession::IMPL_GROUP_SENSOR;
	desc.subgroup = PXCSession::IMPL_SUBGROUP_VIDEO_CAPTURE;
	if (session->QueryImpl(&desc, 0, &desc1) < PXC_STATUS_NO_ERROR) return false;

	PXCCapture * capture;
    pxcStatus status = session->CreateImpl<PXCCapture>(&desc1, &capture);
    if(status != PXC_STATUS_NO_ERROR){
        QMessageBox::critical(NULL,"FATAL ERROR", "Intel RealSense device not plugged?\n(CreateImpl<PXCCapture> failed)");
        exit(0);
    }

    PXCCapture::Device* device;
	device = capture->CreateDevice(0);
	projection = device->CreateProjection();

	this->initialized = true;
	return true;
}

SensorRealSense::~SensorRealSense() {
	std::cout << "~SensorRealSense()" << std::endl;

	if (!initialized) return;
	// TODO: stop sensor 

}

bool SensorRealSense::spin_wait_for_data(Scalar timeout_seconds) {

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

bool SensorRealSense::fetch_streams(DataFrame &frame) {
	if (initialized == false) this->initialize();	

    if(frame.depth.empty())
        frame.depth = cv::Mat(cv::Size(D_width/2, D_height/2), CV_16UC1, cv::Scalar(0));
    if(frame.color.empty())
        frame.color = cv::Mat(cv::Size(D_width/2, D_height/2), CV_8UC3, cv::Scalar(0, 0, 0));

    if (sense_manager->AcquireFrame(true) < PXC_STATUS_NO_ERROR) return false;
    PXCCapture::Sample *sample = sense_manager->QuerySample();

    /// Depth
    {
        sample->depth->AcquireAccess(PXCImage::ACCESS_READ_WRITE, PXCImage::PIXEL_FORMAT_DEPTH, &depth_buffer);
        unsigned short* data = ((unsigned short *)depth_buffer.planes[0]);
        for (int y=0, y_sub=0; y_sub<camera->height(); y+=2, y_sub++) {
            for (int x=0, x_sub=0; x_sub<camera->width(); x+=2, x_sub++) {
                frame.depth.at<unsigned short>(y_sub, x_sub) = data[y*D_width +(D_width-x-1)];
            }
        }
        sample->depth->ReleaseAccess(&depth_buffer);
        ///--- Postprocess
        // cv::medianBlur(depth_cv, depth_cv, 5);
    }

    /// Color
    {
        PXCImage * sync_color_pxc = projection->CreateColorImageMappedToDepth(sample->depth, sample->color);
        sync_color_pxc->AcquireAccess(PXCImage::ACCESS_READ_WRITE, PXCImage::PIXEL_FORMAT_RGB24, &color_buffer);
        for (int y=0, y_sub=0; y_sub<camera->height(); y+=2, y_sub++) {
            for (int x=0, x_sub=0; x_sub<camera->width(); x+=2, x_sub++) {
                unsigned char r = color_buffer.planes[0][y * D_width * 3 + (D_width - x - 1) * 3 + 0];
                unsigned char g = color_buffer.planes[0][y * D_width * 3 + (D_width - x - 1) * 3 + 1];
                unsigned char b = color_buffer.planes[0][y * D_width * 3 + (D_width - x - 1) * 3 + 2];
                frame.color.at<cv::Vec3b>(y_sub, x_sub) = cv::Vec3b(b,g,r); ///< SWAP Channels
            }
        }
        sync_color_pxc->ReleaseAccess(&color_buffer);
    }

	sense_manager->ReleaseFrame();	
	return true;
}

void SensorRealSense::start() {
	if (!initialized)
        this->initialize();
}

void SensorRealSense::stop() {
}
#endif
