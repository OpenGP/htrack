#include "TrivialDetector.h"

#include "util/mylogger.h"
#include "util/opencv_wrapper.h"
#include "tracker/Data/DataFrame.h"
#include "tracker/DataStructure/SkeletonSerializer.h"
#include "tracker/OffscreenRender/OffscreenRenderer.h"
#include "tracker/OffscreenRender/CustomFrameBuffer.h"
#include "tracker/HandFinder/HandFinder.h"
#include "tracker/Worker.h"

bool TrivialDetector::compute_centroid_render(cv::Mat color_render, cv::Mat depth_render, Vector3& retval)
{
    Vector3 tot(0,0,0);
    int count = 0;
    for(int row = 0; row < depth_render.rows; row++){
        for(int col = 0; col < depth_render.cols; col++){
            if(color_render.at<uchar>(row,col)==255)
                continue;
            cv::Vec4f p = depth_render.at<cv::Vec4f>(row, col);
            tot += Vector3(p[0], p[1], p[2]);
            count++;
        }
    }

    if(count==0){
        retval = Vector3(0,0,0);
        return false;
    } else {
        retval = tot / count;
        return true;
    }
}

bool TrivialDetector::compute_centroid_sensor(DataFrame& frame, cv::Mat sensor_silhouette, Vector3& retval)
{
    Vector3 tot(0,0,0);
    int count = 0;
    for(int row=0; row<sensor_silhouette.rows; row++){
        for(int col=0; col<sensor_silhouette.cols; col++){
            if(sensor_silhouette.at<uchar>(row,col)<125)
                continue;
            Vector3 p_sensor = frame.point_at_pixel(col,row,camera);
            tot += p_sensor;
            count++;
        }
    }

    if(count==0){
        retval = Vector3(0,0,0);
        return false;
    } else {
        retval = tot / count;
        return true;
    }
}

void TrivialDetector::copy_rendered_images_to_cpu(){
    // TIMED_SCOPE(timer,"copy_rendered_images_to_cpu()");
    ///--- Depth
    depth_render = offscreenrend->fb->fetch_extra_attachment();
    cv::flip(depth_render, depth_render, 0 /*flip rows*/ );

    ///--- Color
    color_render = offscreenrend->fb->fetch_color_attachment();
    cv::flip(color_render, color_render, 0 /*flip rows*/ );
}

TrivialDetector::TrivialDetector(Camera *camera, OffscreenRenderer *offscreenrend, SkeletonSerializer *skeleton) :
    camera(camera), offscreenrend(offscreenrend), skeleton(skeleton)
{
    CHECK_NOTNULL(camera);
    CHECK_NOTNULL(offscreenrend);
    CHECK_NOTNULL(skeleton);
}

void TrivialDetector::exec(DataFrame& frame, const cv::Mat& sensor_silhouette)
{
    LOG(INFO) << "Worker::initialize_offset";
    bool success = true;
    Vector3 c1;
    Vector3 c2;

    ///--- Make hand in standard pose
    success &= compute_centroid_sensor(frame, sensor_silhouette, /*=*/c2);
    if(!success)
        return;

    ///--- We have data to work on...
    skeleton->reset_hand();

    ///--- Get cloud
    offscreenrend->render_offscreen();
    this->copy_rendered_images_to_cpu();

    success &= compute_centroid_render(this->color_render, this->depth_render, /*=*/c1);

    if(!success){
        LOG(INFO) << "WARNING: init failed";
        return;
    }

    ///--- Apply the transformation
    Vector3 tr = c2-c1;

    ///--- Just to be sure
    assert(!isnan(tr[0]));
    assert(!isnan(tr[1]));
    assert(!isnan(tr[2]));

    // LOG(INFO) << "translation" << tr;
    std::vector<float> thetas = skeleton->getCurrentParameters();
    thetas[0] += tr[0];
    thetas[1] += tr[1];
    thetas[2] += tr[2];
    skeleton->set(thetas);
}

