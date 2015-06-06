#pragma once
#include <thrust/device_vector.h>
#include "tracker/DataStructure/CustomJointInfo.h"

//== NAMESPACE ================================================================
namespace cudax {
//=============================================================================

struct Kinematic{
    /// These hold memory
    thrust::device_vector<CustomJointInfo> D_jointinfos;
    thrust::device_vector<ChainElement>    D_chains;
    /// These are used to access data in the kernel
    CustomJointInfo* jointinfos; ///< @note  device raw pointer
    ChainElement* chains; ///< @note device raw pointer
};

//=============================================================================
} // namespace cudax
//=============================================================================


