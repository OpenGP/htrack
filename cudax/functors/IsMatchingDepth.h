#pragma once
#include "cudax/kernel.h"
#include "IsSilhouette.h"

//== NAMESPACE ================================================================
namespace cudax {
//=============================================================================

//== NAMESPACE ================================================================
namespace hidden {
//=============================================================================

struct IsMatchingDepth{
    float z_th;
    IsMatchingDepth(){
        z_th = settings->depth_z_th; 
    }

    __device__ 
    bool operator()(const int4& off){
        float d_real = (float)SENSOR_DEPTH(off);
        float d_rend = (float)RENDER_DEPTH(off);
        float diff = fabs(d_real - d_rend);
        
/// This is the one used by chen_cvpr14
//#define ENABLE_OCCLUSION_ENERGY
#ifdef ENABLE_OCCLUSION_ENERGY
        if(d_rend<d_real && diff<100 /*mm*/)
            return true;
        else
            return false;
#else
        return (diff <= z_th);
#endif        
    }
};

//=============================================================================
} // namespace hidden
//=============================================================================

struct IsDepthConstraint{
    IsSensorSilhouette test_1;
    hidden::IsMatchingDepth test_2;
    IsDepthConstraint(thrust::device_vector<uchar>& silhouette_sensor) : 
        test_1(silhouette_sensor){}

    __device__
    bool operator()(const int4& off){
        return test_1(off) && test_2(off);
    }
};

//=============================================================================
} // namespace cudax
//=============================================================================
