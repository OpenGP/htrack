#include "LegacyTracker.h"
#include "tracker/Data/DataStream.h"
#include "algorithm/ICP.h"
#include "geometry/Cylinders.h"
#include "util/mylogger.h"
#include "util/openmp_helpers.h"
#include "tracker/HandFinder/HandFinder.h"
#include "tracker/Legacy/algorithm/ICP.h"
#include "tracker/Legacy/util/PostureFile.h"
#include "util/qfile_helper.h"

void LegacyTracker::track(DataFrame& frame){
    int max_rigid_iters = settings->termination_max_rigid_iters;
    int max_num_iters = settings->termination_max_iters;

    assert( settings->icp_enabled );
    handfinder->binary_classification(frame);
    // ::worker->skeleton->reset_hand(); ///< ???
    if(!(handfinder->has_useful_data()))
        return;
    track_icp(frame, max_num_iters, max_rigid_iters);
}

void LegacyTracker::init(Camera *camera, Skeleton *skeleton, Cylinders* cylinders){
    this->camera = camera;
    this->skeleton = skeleton;
    this->cylinders = cylinders;
    icp       = new ICP();
    icp->ik.setLambda(5.0f);
    icp->ik.setClamping(true);
    icp->ik.setPrediction(true);
    icp->ik.setLineSearch(true);

    if(settings->icp_pca_enabled){
        std::string path = local_file_path("fingerfist.mat");
        int bases = settings->icp_pca_bases;
        skeleton->getMapping().doPCA(PostureFile(path).postures, true, bases, 6);
    }
}

void LegacyTracker::track_icp(DataFrame& frame, int iterTotal, int iterRigid){
    openmp::setNumThreads(openmp::NUM_THREADS);

    int s = 3;
    std::vector<Vec3f> points;
    std::vector<Vec3f> wristband;

    for( int y = 0; y < camera->height(); y+=s ){
        for( int x = 0; x < camera->width(); x+=s ){
            Scalar depth = frame.depth_at_pixel(x,y);
            uint sil = (uint) handfinder->sensor_silhouette.at<uchar>(y,x);
            uint wri = (uint) handfinder->sensor_wristband.at<uchar>(y,x);

            if(camera->is_valid(depth)){
                if(sil){
                    Vector3 p = frame.point_at_pixel(x,y,camera);
                    points.push_back(p);
                }
                if(wri && settings->forearm_enabled){
                    Vector3 p = frame.point_at_pixel(x,y,camera);
                    wristband.push_back(p);
                }
            }
        }
    }

    int r = iterRigid;
    int nr = iterTotal - iterRigid;

    if(settings->forearm_enabled){
        icp->alignWithWristband(points, wristband, *cylinders, r, nr);
    } else {
        icp->align(points, *cylinders, r, nr);
    }

    if(settings->icp_pca_enabled){
        icp->adaptPCA(cylinders->getSkeleton());
    }
}

void LegacyTracker::track_icp_rigid(DataFrame& frame, int iterations)
{
    // assert(settings->crop_below_armband);
    int s = 3;
    std::vector<Vec3f> points;

    // TICTOC_BLOCK(timer,"Worker::track_icp_rigid::(extract points)")
    {
        for( int y = 0; y < camera->height(); y+=s ){
            for( int x = 0; x < camera->width(); x+=s ){
                Scalar depth = frame.depth_at_pixel(x,y);
                uint sil = (uint) handfinder->sensor_silhouette.at<uchar>(y,x);
                if(camera->is_valid(depth) && sil){
                    Vector3 p = frame.point_at_pixel(x,y,camera);
                    points.push_back(p);
                }
            }
        }
    }

    // TICTOC_BLOCK(timer,"Worker::track_icp_rigid::(ICP iterations)")
    {
        icp->rigidICP(*cylinders, points, iterations);
    }
}

