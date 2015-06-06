#pragma once
#include "kernel.h"
#include "Kinematic.h"

using namespace cudax;

void kernel_upload_dtform_idxs(int* H_dtform_idxs){
    thrust::copy(H_dtform_idxs, H_dtform_idxs+H_width*H_height, sensor_dtform_idxs->begin());    
}

void kernel_upload_sensor_data(uchar* H_silhouette_sensor)
{
    thrust::copy(H_silhouette_sensor, H_silhouette_sensor+H_width*H_height, silhouette_sensor->begin());
}

void kernel_upload_kinematic(const JointTransformations& H_jointinfos, const KinematicChain& H_kinchains)
{
    kinematic->D_jointinfos.resize(H_jointinfos.size());
    thrust::copy(H_jointinfos.begin(), H_jointinfos.end(), kinematic->D_jointinfos.begin());
    kinematic->D_chains.resize(H_kinchains.size());
    thrust::copy(H_kinchains.begin(), H_kinchains.end(), kinematic->D_chains.begin());

    // printf("#jointinfos: %ld\n", kinematic->D_jointinfos.size());
    // printf("#kinematic: %ld\n", kinematic->D_chains.size());
    
    /// Fetch raw pointers that can be accessed in the kernels
    kinematic->jointinfos = thrust::raw_pointer_cast(&kinematic->D_jointinfos[0]);
    kinematic->chains = thrust::raw_pointer_cast(&kinematic->D_chains[0]);
}

void kernel_upload_cylinders(const float* H_cylinders)
{
    // printf("kernel_upload_cylinders\n");
    thrust::copy(H_cylinders, H_cylinders+SEGMENT_VALUES*SEGMENT_JOINTS, cylinder_segments->begin());
}

