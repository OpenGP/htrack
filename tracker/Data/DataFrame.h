#pragma once
#include "opencv2/core/core.hpp" /// cv::Mat
#include "tracker/Types.h"
#include "Camera.h"

typedef unsigned short DepthPixel;    
typedef cv::Vec3b ColorPixel;

struct DataFrame{
    int id; ///< unique id (not necessarily sequential!!)
    cv::Mat color; ///< CV_8UC3
    cv::Mat depth; ///< CV_16UC1
   
    DataFrame(int id):id(id){}
        
    /// @param horizontal pixel
    /// @param vertical pixel
    /// @param camera parameters to invert the transformation
    Vector3 point_at_pixel(int x, int y, Camera* camera){
        Integer z = depth_at_pixel(x,y);
        return camera->depth_to_world(x,y,z);
    }
    
    Real depth_at_pixel(const Vector2i& pixel /*row,col*/){
        return depth.at<DepthPixel>(pixel[0],pixel[1]);        
    }
    
    Real depth_at_pixel(int x, int y){
        /// Access is done as (row,colum)
        return depth.at<DepthPixel>(y,x);
    }
};
