#pragma once
#include "tracker/ForwardDeclarations.h"
#include "Energy.h"
#include "tracker/Types.h"
#include <vector>

namespace energy{

class Damping : public Energy{
    SkeletonSerializer* skeleton = NULL;

    struct Settings{
        Scalar intertial_damping = 300;
        Scalar intertial_damping_translation = 1;
    } _settings;
    Settings*const settings = &_settings;

public:
    void init(SkeletonSerializer* skeleton);
    void track(LinearSystem& system);
};

}
