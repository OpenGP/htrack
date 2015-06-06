#pragma once
#include "tracker/Types.h"
#include "tracker/ForwardDeclarations.h"
#include "util/opencv_wrapper.h"

class TrivialDetector{
private:
    Camera*const camera;
    OffscreenRenderer*const offscreenrend;
    SkeletonSerializer*const skeleton;
private:
    cv::Mat color_render; ///< [0..N] joint ID, 255 is background
    cv::Mat depth_render; ///< 3D points in camera space

public:
    TrivialDetector(Camera* camera, OffscreenRenderer* offscreenrend, SkeletonSerializer* skeleton);
    void exec(DataFrame &frame, const cv::Mat& sensor_silhouette);
private:
    void copy_rendered_images_to_cpu();
    bool compute_centroid_sensor(DataFrame &frame, cv::Mat sensor_silhouette, Vector3 &retval);
    bool compute_centroid_render(cv::Mat color_render, cv::Mat depth_render, Vector3 &retval);
};
