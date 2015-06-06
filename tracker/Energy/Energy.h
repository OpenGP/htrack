#pragma once
#include "tracker/Types.h"

struct TrackingError{
    float _3D;
    float _2D;
    static TrackingError infinity(){ return {inf(), inf()}; }
};

namespace energy{

class Energy{
protected:
    bool safety_check = false;
    bool has_nan(LinearSystem &system);

public:
    static void rigid_only(LinearSystem& system);
    static VectorN solve(LinearSystem& system);
};

}

