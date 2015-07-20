#pragma once
#include <queue>
#include <fstream>
#include "tracker/Types.h"

class TrackingMonitor{
private:
    std::queue<float> failure_metric_history;
    float failure_metric_sum = 0;

public:

    struct Settings{
        int moving_window_size = 10;
    } _settings;
    Settings*const settings = &_settings;

    bool is_failure_frame(float pull_error, float push_error) {

        float threshold = 0.5;
        Vector3 gamma = Vector3::Zero(3);
        //#if defined(REALSENSE) //
        //gamma = Vector3(23.57251f, 8.20843f, -4.07201f);
        //#endif
        //#if defined(SOFTKIN)
        gamma = Vector3(0.0f, 25.708f, -1.7853f);
        //#endif

        /*ofstream errors_file;
            if (!errors_file.is_open()) errors_file.open("/home/anastasia/Desktop/errors.txt", ios::app);
            errors_file << push_error << " " << pull_error << endl;*/


        Vector3 x = Vector3(1.0, push_error, pull_error);
        /// Failure: 0 , Success: 1
        float metric = 1.0 / (1.0 + pow((float)exp(1.0), -x.transpose() * gamma));
        if (metric < threshold) metric = 0;
        else metric = 1;

        failure_metric_history.push(metric);
        failure_metric_sum = failure_metric_sum + metric;
        while (failure_metric_history.size() > settings->moving_window_size) {
            failure_metric_sum = failure_metric_sum - failure_metric_history.front();
            failure_metric_history.pop();
        }

        if ((failure_metric_history.size() == settings->moving_window_size) && (failure_metric_sum == 0))
            return true;
        return false;
    }
};

