#pragma once
#include "tracker/ForwardDeclarations.h"
#include "tracker/Types.h"
#include "tracker/Energy/Fitting/Settings.h"
struct MappedResource;

namespace energy{
class Fitting{
protected:
    Camera* camera = NULL;
    OffscreenRenderer* offscreenrend = NULL;
    DepthTexture16UC1* sensor_depth_texture = NULL;
    SkeletonSerializer* skeleton = NULL;
    Cylinders* cylinders = NULL;
    HandFinder* handfinder = NULL;
protected:
    fitting::Settings _settings;
    fitting::Settings*const settings = &_settings;

public:
#ifdef WITH_CUDA
    void track(DataFrame &frame, LinearSystem &sys, bool rigid_only, bool eval_error, float &push_error, float &pull_error);
    void init(Worker* worker);
    void cleanup();
#else
    void track(DataFrame &frame, LinearSystem &sys, bool rigid_only, bool eval_error, float &push_error, float &pull_error){}
    void init(Worker* worker){}
    void cleanup(){}
#endif
};
} /// energy::
