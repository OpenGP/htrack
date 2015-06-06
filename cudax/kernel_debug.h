#pragma once
#include "kernel.h"
#include <thrust/device_vector.h>
#include <thrust/copy.h>

using namespace cudax;

//-----------------------------------------------------------------//

struct ConstraintTypeFunctor{
    uchar* opencv_image_d_raw;
    int2* cnstr_indexes;
    float* e_raw;

    ConstraintTypeFunctor(uchar *opencv_image_d_raw, float* e_raw){
        this->opencv_image_d_raw = opencv_image_d_raw;
        this->cnstr_indexes = pixel_indexer->cnstr_indexes;
        this->e_raw = e_raw;
    }

    __device__ int type(const MeshGrid::Elem& off){ return cnstr_indexes[off.z].x; }
    __device__ int constraint_index(const MeshGrid::Elem& off){ return cnstr_indexes[off.z].y; }   

    __device__
    void operator()(MeshGrid::Elem& off){
        if(type(off) == PixelType::CONSTRAINT_SILHO){
            float r = e_raw[constraint_index(off)];
            if(r == 0){
                opencv_image_d_raw[off.z*3+0] = 255;
            } else {
                opencv_image_d_raw[off.z*3+1] = 255;
                opencv_image_d_raw[off.z*3+2] = 255;
            }
            return;
        }
        if(type(off) == PixelType::CONSTRAINT_DEPTH){
            opencv_image_d_raw[off.z*3+1] = 255;
            return;
        }
        if(type(off) == PixelType::CONSTRAINT_EXTRA_PULL){
            opencv_image_d_raw[off.z*3+2] = 255;
            return;
        }
        if(type(off) == PixelType::CONSTRAINT_EXTRA_PUSH){
            opencv_image_d_raw[off.z*3+2] = 125;
            return;
        }
    }
};

void kernel_constraint_type_image(uchar *opencv_image, int w, int h){
    thrust::device_vector<uchar> opencv_image_d(opencv_image, opencv_image+3*(w*h));
    uchar *opencv_image_d_raw = thrust::raw_pointer_cast(opencv_image_d.data());
    ConstraintTypeFunctor functor(opencv_image_d_raw, thrust::raw_pointer_cast(e->data()));
    thrust::for_each(meshgrid->begin(), meshgrid->end(), functor);
    thrust::copy(opencv_image_d.begin(), opencv_image_d.end(), opencv_image);
}

//-----------------------------------------------------------------//

struct SetZeroFunctor{
    J_row* J_raw;
    SetZeroFunctor(J_row* J_raw):J_raw(J_raw){}
    __device__
    void operator()(const int i){
        J_row &Ji = *(J_raw + i);
        for(int j=6; j<NUM_THETAS; ++j){
            /*((J_row)(*J)[i]).data[j] = 0;;*/
            Ji.data[j] = 0;
        }
    }
};

void kernel_simplify_jacobian(int n_total){
    thrust::counting_iterator<int> it(0);
    thrust::for_each(it, it+n_total, SetZeroFunctor(thrust::raw_pointer_cast(J->data())));
}

//-----------------------------------------------------------------//

void kernel_store_closedform_corresp(int num)
{
    if(num <= 0){
        //disable
        if(debug_closedform_correspondences) delete debug_closedform_correspondences;
        store_corresps = false;
    } else if(!store_corresps){
        // initialize
        debug_closedform_correspondences = new thrust::device_vector<float>(num, -111);
        store_corresps = true;
    } else if(debug_closedform_correspondences->size() != num) {
        // re-initialize
        delete debug_closedform_correspondences;
        debug_closedform_correspondences = new thrust::device_vector<float>(num, -111);
    }
}

void kernel_copy_closedform_corresp(thrust::host_vector<float> &H_closedform_corrs)
{
    if(store_corresps){
        H_closedform_corrs.resize(debug_closedform_correspondences->size());
        H_closedform_corrs = *debug_closedform_correspondences;
    }
}

