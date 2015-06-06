#pragma once
#include "tracker/ForwardDeclarations.h"
#include "Energy.h"
#include "tracker/Types.h"
#include <vector>

namespace energy{

class Wristband : public Energy{
    Camera* camera;
    SkeletonSerializer* skeleton;
    HandFinder* handfinder;

/// @{ settings
    bool classifier_enable = true;
    float classifier_weight = 1e5;
    bool classifier_show_axis = false;
    bool classifier_temporal = false;
/// @}

public:
    void init(Camera*camera, SkeletonSerializer* skeleton, HandFinder* handfinder);
    void track(LinearSystem& system);
};

} /// energy::
