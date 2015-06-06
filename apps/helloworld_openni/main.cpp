#include <iostream>
#include "util/tictoc.h"
#include "util/opencv_wrapper.h"
#include "tracker/Sensor/Sensor.h"
#include "tracker/Data/DataFrame.h"

int main(int /*argc*/, char ** /*argv*/){   
    Camera* camera = new Camera(QVGA, 60 /*FPS*/);
    Sensor* sensor = new SensorOpenNI(camera);

    ///--- start the sensor    
    sensor->start();
    bool success = sensor->spin_wait_for_data(5 /*seconds*/);
    assert(success);
    
    DataFrame frame(0);
    for(int i=0; i<30 /*seconds*/ *60 /*60FPS*/; i++){
        bool success = sensor->fetch_streams(frame);
        assert(success);
        // cv::normalize(frame.color, frame.color, 0, 255, cv::NORM_MINMAX);
        
        ///--- Show color
        cvtColor(frame.color, frame.color, CV_BGRA2RGBA);
        cv::imshow("color", frame.color);
        
        ///--- Show depth
        cv::normalize(frame.depth, frame.depth, 0, USHRT_MAX, cv::NORM_MINMAX);
        cv::imshow("depth", frame.depth);
        
        ///--- Wait a sec
        cv::waitKey(15 /*ms -> 60fps*/);
    }
    
    std::cout << "finished!" << std::endl;
    return 0;
}
