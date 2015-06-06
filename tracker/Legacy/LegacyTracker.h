#pragma once
#include "tracker/ForwardDeclarations.h"
#include "tracker/Types.h"

#if 0
    /// TODO: re-wire legacy tracker [Shroder ICRA'14]
    if(settings->icp_enabled){
        track_icp(frame, 1, 0);
        return;//don't do anything else
    }
#endif

class LegacyTracker{
    Camera* camera = NULL;
    Cylinders* cylinders = NULL;
    HandFinder* handfinder = NULL;
    Skeleton* skeleton = NULL;
    ICP* icp = NULL; ///< TODO: delete appropriately

/// @{ Settings
public:
    struct Settings{
        int termination_max_iters = 5;
        int termination_max_rigid_iters = 1;
        bool forearm_enabled = false;
        bool icp_enabled = false;
        bool icp_pca_enabled = false;
        int icp_pca_bases=6;
    } _settings;
    Settings*const settings = &_settings;
/// @}

public:
    void init(Camera* camera, Skeleton* skeleton, Cylinders *cylinders);
    void track(DataFrame& frame);
private:
    void track_icp_rigid(DataFrame &frame, int iterations);
    void track_icp(DataFrame &frame, int iterTotal, int iterRigid);
};
