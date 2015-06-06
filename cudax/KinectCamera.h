#pragma once
#include <thrust/device_vector.h>
#include "cuda_runtime.h"
#include "cuda_glm.h"

//== NAMESPACE ================================================================
namespace cudax {
//=============================================================================

struct KinectCamera{
    thrust::device_vector<glm::mat3x3> inv_proj_matrix; ///< only containts one
    glm::mat3x3* D_mat_ptr; ///< device pointer to its content
    
    KinectCamera(const float* H_inv_proj_matrix){
        inv_proj_matrix.resize(1);
        thrust::device_ptr<glm::mat3x3> mat = &inv_proj_matrix[0];
        D_mat_ptr = thrust::raw_pointer_cast(mat);
        cudaMemcpy((void*) D_mat_ptr, H_inv_proj_matrix, 9*sizeof(float), cudaMemcpyHostToDevice);
    }
    const glm::mat3x3& D_inv_proj_matrix(){
        return (*D_mat_ptr);
    }
};

//=============================================================================
} // namespace cudax
//=============================================================================
