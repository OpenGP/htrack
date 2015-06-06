#pragma once
#include "cudax/kernel.h"

//== NAMESPACE ================================================================
namespace cudax {
//=============================================================================

struct IsTopLeft3x3Block : public thrust::unary_function<bool, MeshGrid::Elem>{
    __device__
    bool operator()(MeshGrid::Elem off){
        return ((off.x<3) && (off.y<3));
    }
};

//=============================================================================
} // namespace cudax
//=============================================================================




