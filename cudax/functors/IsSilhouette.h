#pragma once
#include "cudax/kernel.h"

//== NAMESPACE ================================================================
namespace cudax {
//=============================================================================

struct IsRenderedSilhouette{
    __device__
    bool operator()(const int4& off){
        return (tex2D(color_tex, off.x, off.y)<255);
    }
    __device__
    bool operator()(int x, int y){
        return (tex2D(color_tex, x, y)<255);
    }
};

struct IsSensorSilhouette{
    uchar* silhouette_sensor;
    IsSensorSilhouette(thrust::device_vector<uchar>& silhouette_sensor){
        this->silhouette_sensor = thrust::raw_pointer_cast(&silhouette_sensor[0]);
    }

    __device__
    bool operator()(const int4& off){
        return (silhouette_sensor[off.z] > 125);
    }
};

//=============================================================================
} // namespace cudax
//=============================================================================
