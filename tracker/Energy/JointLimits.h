#pragma once
#include "tracker/ForwardDeclarations.h"
#include "Energy.h"
#include "tracker/Types.h"
#include <vector>

namespace energy{

class JointLimits : public Energy{
    SkeletonSerializer* skeleton = NULL;
    bool jointlimits_enable = true;
    float jointlimits_weight = 10000000;
    bool jointlimits_show_constraints = false;

public:
    void init(SkeletonSerializer* skeleton);
    void track(LinearSystem& sys, const std::vector<Scalar> &theta_0);
};

}
