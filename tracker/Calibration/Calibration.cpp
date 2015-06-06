#include "Calibration.h"

#include "util/qfile_helper.h"
#include "tracker/Legacy/geometry/Cylinders.h"
#include "tracker/DataStructure/SkeletonSerializer.h"
#include "tracker/Worker.h"
#include "Skeleton_IO.h"
#include <fstream>

inline bool file_exists(const std::string& name){
    ifstream f(name.c_str());
    if(f.good()){
        f.close();
        return true;
    } else {
        f.close();
        return false;
    }
}

Calibration::Calibration(Worker *worker) :
    skeleton(worker->skeleton),
    cylinders(worker->cylinders) {}

void Calibration::save(const string &filename){
    if(filename.empty()){
        std::cout << "No calibration file specified." << std::endl;
        return;
    }

    // assume neutral posture
    vector<float> ps = skeleton->getCurrentParameters();
    skeleton->reset();

    // save skeleton
    Skeleton_IO(skeleton).write(filename);

    // undo neutral posture
    skeleton->set(ps);

    // save cylinders
    std::ofstream cf(filename, std::ios_base::app);
    const vector<Segment> &segs = cylinders->getSegments();
    for(size_t i = 0; i < segs.size(); ++i){
        cf << i << " "
           << segs[i].radius1 << " "
           << segs[i].radius2 << " "
           << segs[i].radius1_y << " "
           << segs[i].radius2_y << endl;
    }
    cf.close();

    cout << "Saved calibrated model to " << filename << endl;
}

void Calibration::load(const std::string &filename)
{
    if(filename.empty()){
        std::cout << "No calibration file specified." << std::endl;
        return;
    }

    if(!file_exists(filename)){
        std::cout << "File " << filename << " not found." << std::endl;
        return;
    }

    // skeleton
    Skeleton_IO sf = Skeleton_IO(filename);

    // transfer rest pose
    vector<float> ps = skeleton->getCurrentParameters();
    const aligned_map<string, Mat4f>::type &ts =
        sf.skeleton()->getInitialTransformations();
    for(auto t : ts)
        skeleton->setInitialTransformation(t.first, t.second);

    // reset model
    skeleton->setScaleValue(skeleton->getScaleMagnitude());
    skeleton->resetPosture();
    skeleton->set(ps);

    // cylinders
    std::ifstream cf(filename);
    bool end_of_skeleton = false;
    vector<Segment> &segs = cylinders->getSegments();
    while(!cf.eof()){
        if(!end_of_skeleton){
            std::string line;
            std::getline(cf, line);
            if(!line.compare("end_of_skeleton")){
                end_of_skeleton = true;
            }
        } else {
            size_t index;
            cf >> index;
            if(index < segs.size()){
                cf >> segs[index].radius1;
                cf >> segs[index].radius2;
                cf >> segs[index].radius1_y;
                cf >> segs[index].radius2_y;
            } else {
                float _; // ignore this
                cf >> _ >> _ >> _ >> _;
            }
        }
    }
    cf.close();
    cout << "Loaded calibrated model from " << filename << endl;
}

void Calibration::autoload(){
    std::cout << "Calibration::autoload()" << std::endl;
    std::string path = local_file_path("default.calib");
    load(path);
}

void Calibration::update_radius_helper(Cylinders *cylinders, SkeletonSerializer *skeleton){
    int iw = skeleton->getID("Hand");
    int ii = skeleton->getID("HandIndex1");
    int ip = skeleton->getID("HandPinky1");

    Segment &sw = cylinders->getSegmentByID(iw);
    Segment &si = cylinders->getSegmentByID(ii);
    Segment &sp = cylinders->getSegmentByID(ip);

    Vector3 ti = skeleton->getJoint("HandIndex1")->getGlobalTranslation();
    Vector3 tp = skeleton->getJoint("HandPinky1")->getGlobalTranslation();

    float s = skeleton->getScaleMagnitude();
    float d = (ti-tp).norm();
    float r2 = sw.radius2;
    float r2y = sw.radius2_y;
    float rad2_ = (d + si.radius1*s + sp.radius1*s) * 0.5f + 1.5f * s;
    // float rad2y = r2y/r2 * rad2_;

    float sr = sw.radius2;
    sw.radius2 = rad2_/s;
    //sw.radius2_y = rad2y/s;
    sr /= sw.radius2;
    sw.radius1 /= sr;
    //sw.radius1_y /= sr;
}
