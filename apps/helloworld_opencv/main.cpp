#include <iostream>
#include <fstream>
#include "stdlib.h"
#include "opencv2/core/core.hpp"
#include "opencv2/core/types_c.h" ///< CV_RGB2GRAY
#include "opencv2/imgproc/imgproc.hpp" ///< cvtColor
#include "opencv2/highgui/highgui.hpp"

int main(int, char **) {
    cv::Mat image = cv::imread("input.png");
    std::cout << "image size: " << image.rows << " " << image.cols << std::endl;
    assert(image.rows > 0);
    assert(image.cols > 0);
    cv::cvtColor(image, image, CV_RGB2GRAY);
    cv::imshow("input", image);
    cv::waitKey(0);
    exit(0);
}
