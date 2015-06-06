#pragma once

#include <vector>
#include "DataStream.h"
#include "MathUtils.h"
#include "Util.h"
#include "util/opencv_wrapper.h"

class Detector
{
public:

    Camera* const camera;
    int sampling;

    std::vector<Vec3f> left_points;

public:

    Detector(Camera *s) : camera(s){
        sampling = 4;
    }

    void detect(DataFrame &f)
    {
#if 1
        // TODO implement proper hand detection
        //filter(f, cv::Scalar(0, .23*255, 0), cv::Scalar(35, .68*255, 255));
        filter(f, cv::Scalar(0,56,113), cv::Scalar(179,174,255));

        //cv::Scalar blueMin(100,150,0);
        //cv::Scalar blueMax(120,256,256);
        //filter(f, blueMin, blueMax);

        //cv::Scalar red1Min(0,150,0);
        //cv::Scalar red1Max(20,256,256);
        //filter(f, red1Min, red1Max);

        //cv::Scalar red2Min(170,150,0);
        //cv::Scalar red2Max(180,256,256);
        //filter(f, red2Min, red2Max);

#else
        // subsample 3d point constraints
        left_points.clear();
        for(int y = 0; y < f.depth.rows; y += sampling)
        {
            for(int x = 0; x < f.depth.cols; x += sampling)
            {
                Scalar depth = f.depth_at_pixel(x,y);

                if(depth > 100 && depth < 500)
                {
                    Vec3f p = f.point_at_pixel(x, y, stream->camera());
                    left_points.push_back(p);
                }
            }
        }
#endif
    }

    void filter(DataFrame &f, const cv::Scalar &min, const cv::Scalar &max)
    {
        bool showim = false;

        //-- color classification

        cv::Mat hsv, bw, bw_rgb;
        cv::cvtColor(f.color, hsv, CV_RGB2HSV);
        cv::inRange(hsv, min, max, bw);
        cv::cvtColor(bw, bw_rgb, CV_GRAY2BGR);
        if(showim && bw_rgb.data) cv::imshow("src", bw_rgb);

        //-- blob detection

        //cv::RNG rng(12345);
        cv::Mat output;

        vector<vector<cv::Point> > contours;
        vector<cv::Vec4i> hierarchy;

        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(4,4));
        cv::morphologyEx(bw, output, cv::MORPH_OPEN, kernel);

        //cv::findContours(output, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
        cv::findContours(output, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
        //cv::findContours(output, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
        //cv::findContours(output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

        //cv::Mat drawing = cv::Mat::zeros(output.size(), CV_8UC3);
        cv::Mat drawing = cv::Mat::zeros(output.size(), CV_8UC1);
        for(int i = 0; i < contours.size(); ++i)
        {
            //cv::Scalar color = cv::Scalar(rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255));
            //cv::drawContours(drawing, contours, i, color, 2, 8, hierarchy, 0, cv::Point());
            cv::Scalar color = cv::Scalar(255,255,255);
            cv::drawContours(drawing, contours, i, color, CV_FILLED);
        }

        if(showim && drawing.data) cv::imshow("contours", drawing);

        //-- depth/color mask

        cv::Mat dep;
        cv::inRange(f.depth, 100, 500, dep);
        if(showim && dep.data) cv::imshow("dep", dep);

        // TODO offset between color and depth images (f.COL2DEPTHOFFSET)
        cv::Mat mask;
        cv::bitwise_and(drawing, dep, mask);
        if(showim && mask.data) cv::imshow("mask", mask);

        //-- constraint sampling

        left_points.clear();
        for(int y = 0; y < f.depth.rows; y += sampling)
        {
            for(int x = 0; x < f.depth.cols; x += sampling)
            {
                Scalar depth = f.depth_at_pixel(x,y);

                if(mask.at<uchar>(y,x) != 0) // access as (row,column)
                {
                    Vec3f p = f.point_at_pixel(x, y, *camera);
                    left_points.push_back(p);
                }
            }
        }

        //--
    }
};

