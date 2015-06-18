#include "HandFinder.h"

#include <numeric> ///< std::iota
#include <fstream> ///< ifstream
#include "util/mylogger.h"
#include "util/opencv_wrapper.h"
#include "util/qfile_helper.h"
#include "tracker/Worker.h"
#include "tracker/Data/DataFrame.h"
#include "tracker/Data/DataStream.h"
#include "tracker/Detection/TrivialDetector.h"
#include "tracker/DataStructure/SkeletonSerializer.h"
#include "tracker/Legacy/util/Util.h"
#include "./connectedComponents.h" ///< only declared in opencv3

#include "tracker/TwSettings.h"

HandFinder::HandFinder(Camera *camera, TrivialDetector* trivial_detector) : camera(camera), trivial_detector(trivial_detector){
    CHECK_NOTNULL(camera);
    CHECK_NOTNULL(trivial_detector);

    tw_settings->tw_add(settings->show_hand, "show_hand", "group=HandFinder");
    tw_settings->tw_add(settings->show_wband, "show_wband", "group=HandFinder");
    tw_settings->tw_add(settings->wband_size, "wband_size", "group=HandFinder");
    tw_settings->tw_add(settings->depth_range, "depth_range", "group=HandFinder");

#ifdef TODO_TWEAK_WRISTBAND_COLOR
     // TwDefine(" Settings/classifier_hsv_min colormode=hls ");
     TwAddVarRW(tw_settings->anttweakbar(), "rgb_min", TW_TYPE_COLOR3F,  &_settings.hsv_min.data, "group=HandFinder");
     TwAddVarRW(tw_settings->anttweakbar(), "rgb_max", TW_TYPE_COLOR3F,  &_settings.hsv_max.data, "group=HandFinder");
#endif

    std::string path = local_file_path("wristband.txt",true/*exit*/);
    if(!path.empty()){
        std::cout << "Reading Wristband Colors from: " << path << std::endl;
        ifstream myfile(path);
        std::string dump;
        myfile >> dump; ///< "hsv_min:"
        myfile >> settings->hsv_min[0];
        myfile >> settings->hsv_min[1];
        myfile >> settings->hsv_min[2];
        myfile >> dump; ///< "hsv_max:"
        myfile >> settings->hsv_max[0];
        myfile >> settings->hsv_max[1];
        myfile >> settings->hsv_max[2];
        std::cout << "  hsv_min: " << settings->hsv_min << std::endl;
        std::cout << "  hsv_max: " << settings->hsv_max << std::endl;
    }
}

void HandFinder::binary_classification(DataFrame& frame) {
    cv::Mat& mask_wristband = this->sensor_wristband;
    static int previous_frame_id = -1;
    if(previous_frame_id==frame.id)
        return;
    previous_frame_id = frame.id;
    _wristband_found = false;

    // LOG(INFO) << "Worker::binary_classification";
    // TICTOC_SCOPE(timer, "Worker::binary_classification");

    cv::Mat& color = frame.color;
    cv::Mat& depth = frame.depth;

#if 0
    ///--- SPECIAL CASES TO TEST DATASETS
    {
        if(datastream->is_tompson() || datastream->is_colorglove())
        {
            sensor_silhouette = (frame.depth>0);
            // LOG(INFO) << cv::type2str(sensor_silhouette.type());
            has_useful_data = true;
            return;
        }
        if(datastream->is_tang() || datastream->is_chen() || datastream->is_sridhar()){
            sensor_silhouette = (frame.depth>0) & (frame.depth<camera->zFar());
            // LOG(INFO) << cv::type2str(sensor_silhouette.type());
            has_useful_data = true;
            return;
        }
        if(datastream->is_synthetic())
        {
            sensor_silhouette = (frame.color<255);
            cvtColor(sensor_silhouette, sensor_silhouette, CV_RGB2GRAY);
           // LOG(INFO) << cv::type2str(sensor_silhouette.type());
            has_useful_data = true;
            return;
        }
    }
#endif

    ///--- Fetch from settings
    cv::Scalar hsv_min = settings->hsv_min;
    cv::Scalar hsv_max = settings->hsv_max;
    Scalar wband_size = _settings.wband_size;
    Scalar depth_range= _settings.depth_range;

    ///--- We look for wristband up to here...
#if 1
    Scalar depth_farplane = camera->zFar();
#else
    // Scalar depth_farplane= settings->scalar("classifier/depth_farplane");
    Scalar depth_farplane = 500;
    if(camera->mode() == Intel)
        depth_farplane = 500;
    if(camera->mode() == QVGA)
        depth_farplane = 750;
#endif


#if 0
    ///--- Ovverride wristband color for legacy sequences
    if(datastream->is_legacy_maschroe()){
        LOG(INFO) << "LEGACY";
        hsv_min = cv::Scalar(108, 146, 34);
        hsv_max = cv::Scalar(121, 255, 255);
        wband_size=30;
        depth_range=150;
        depth_farplane=600;
    }
#endif

    Scalar crop_radius = 150;
#if 0
    if(datastream->is_melax())
        crop_radius = 120;
#endif

    ///--- Allocated once
    static cv::Mat color_hsv;
    static cv::Mat in_z_range;

    // TIMED_BLOCK(timer,"Worker_classify::(convert to HSV)")
    {
        cv::cvtColor(color, color_hsv, CV_RGB2HSV);
        cv::inRange(color_hsv, hsv_min, hsv_max, /*=*/ mask_wristband);
        cv::inRange(depth, camera->zNear(), depth_farplane /*mm*/, /*=*/ in_z_range);
        cv::bitwise_and(mask_wristband, in_z_range, mask_wristband);
        // cv::imshow("mask_wristband (pre)", mask_wristband);
    }

    // TIMED_BLOCK(timer,"Worker_classify::(robust wrist)")
    {
        cv::Mat labels, stats, centroids;
        int num_components = cv::connectedComponentsWithStats(mask_wristband, labels, stats, centroids, 4 /*connectivity={4,8}*/);
        // for(int row=0; row<num_components; row++){ printf("component[%d]: %d\n", row, stats.at<int>(row,cv::CC_STAT_AREA)); }

        ///--- Generate array to sort
        std::vector< int > to_sort(num_components);
        std::iota(to_sort.begin(), to_sort.end(), 0 /*start from*/);
        // std::copy(to_sort.begin(), to_sort.end(), std::ostream_iterator<int>(std::cout, " "));

        ///--- Sort accoding to area
        auto lambda = [stats](int i1, int i2){
            int area1 = stats.at<int>(i1,cv::CC_STAT_AREA);
            int area2 = stats.at<int>(i2,cv::CC_STAT_AREA);
            return area1>area2;
        };
        std::sort(to_sort.begin(), to_sort.end(), lambda);

        if(num_components<2 /*not found anything beyond background*/){
            // LOG(INFO) << "!!!WARNING not found anything useful";
            _has_useful_data = false;
        }
        else
        {
            if(_has_useful_data==false){
                // std::cout << "NEW useful data => reinit" << std::endl;
                trivial_detector->exec(frame, sensor_silhouette);
            }
            _has_useful_data = true;

            // printf("picked component# %d\n", to_sort[1]);
            ///--- Select 2nd biggest component
            mask_wristband = (labels==to_sort[1]);
            _wristband_found = true;
        }
    }

    if( _settings.show_wband )
        cv::imshow("show_wband", mask_wristband);
    else
        cv::destroyWindow("show_wband");

    // TIMED_BLOCK(timer,"Worker_classify::(crop at wrist depth)")
    {
        ///--- Extract wristband average depth
        std::pair<float, int> avg;
        for (int row = 0; row < mask_wristband.rows; ++row) {
            for (int col = 0; col < mask_wristband.cols; ++col) {
                float depth_wrist = depth.at<ushort>(row,col);
                if(mask_wristband.at<uchar>(row,col)==255){
                     if(camera->is_valid(depth_wrist)){
                         avg.first += depth_wrist;
                         avg.second++;
                     }
                 }
            }
        }
        ushort depth_wrist = (avg.second==0) ? camera->zNear() : avg.first / avg.second;
        // cout << "depth_wrist" << depth_wrist << endl;


        ///--- First just extract pixels at the depth range of the wrist
        cv::inRange(depth, depth_wrist-depth_range, /*mm*/
                           depth_wrist+depth_range, /*mm*/
                           sensor_silhouette /*=*/);
    }

    // cv::imshow("sensor_silhouette (before)", sensor_silhouette);

    _wband_center = Vector3(0,0,0);
    _wband_dir = Vector3(0,0,-1);
    // TIMED_BLOCK(timer,"Worker_classify::(PCA)")
    {
        ///--- Compute MEAN
        int counter = 0;
        for (int row = 0; row < mask_wristband.rows; ++row){
            for (int col = 0; col < mask_wristband.cols; ++col){
                if(mask_wristband.at<uchar>(row,col)!=255) continue;
                _wband_center += frame.point_at_pixel(col,row,camera);
                counter ++;
            }
        }
        _wband_center /= counter;
        std::vector<Vector3> pts; pts.push_back(_wband_center);

        ///--- Compute Covariance
        static std::vector<Vector3> points_pca;
        points_pca.reserve(100000);
        points_pca.clear();
        for (int row = 0; row < sensor_silhouette.rows; ++row){
            for (int col = 0; col < sensor_silhouette.cols; ++col){
                if(sensor_silhouette.at<uchar>(row,col)!=255) continue;
                Vector3 p_pixel = frame.point_at_pixel(col,row,camera);
                if((p_pixel-_wband_center).norm()<100){
                    // sensor_silhouette.at<uchar>(row,col) = 255;
                    points_pca.push_back(p_pixel);
                } else {
                    // sensor_silhouette.at<uchar>(row,col) = 0;
                }
            }
        }
        if (points_pca.size() == 0) return;
        ///--- Compute PCA
        Eigen::Map<Matrix_3xN> points_mat(points_pca[0].data(), 3, points_pca.size() );
        // Vector3 mean = points_mat.rowwise().mean();
        for(int i : {0,1,2})
            points_mat.row(i).array() -= _wband_center(i);
        Matrix3 cov = points_mat*points_mat.adjoint();
        Eigen::SelfAdjointEigenSolver<Matrix3> eig(cov);
        _wband_dir = eig.eigenvectors().col(2);

        ///--- Allow wrist to point downward
        if(_wband_dir.y()<0)
            _wband_dir = -_wband_dir;
    }

    // cv::imshow("sensor_silhouette", sensor_silhouette);

    // TIMED_BLOCK(timer,"Worker_classify::(in sphere)")
    {
        Scalar crop_radius_sq = crop_radius*crop_radius;
        Vector3 crop_center = _wband_center + _wband_dir*( crop_radius + wband_size /*mm*/);
        for (int row = 0; row < sensor_silhouette.rows; ++row){
            for (int col = 0; col < sensor_silhouette.cols; ++col){
                if(sensor_silhouette.at<uchar>(row,col)!=255) continue;

                Vector3 p_pixel = frame.point_at_pixel(col,row,camera);
                if((p_pixel-crop_center).squaredNorm() < crop_radius_sq)
                    sensor_silhouette.at<uchar>(row,col) = 255;
                else
                    sensor_silhouette.at<uchar>(row,col) = 0;
            }
        }
    }

    if(_settings.show_hand){
        cv::imshow("show_hand", sensor_silhouette);
    } else {
        cv::destroyWindow("show_hand");
    }
}

