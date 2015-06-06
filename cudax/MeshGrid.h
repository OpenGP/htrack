#pragma once
#include <cuda_runtime.h>
#include "thrust/device_vector.h"

//== NAMESPACE ================================================================
namespace cudax {
//=============================================================================

/// Device array containing MATLAB meshgrid like data
/// int4:
/// x) i: row
/// y) j: col
/// z) off: linear array iterator (i*width+j)
/// w) id: recycled for computation later (constraint id)
struct MeshGrid{
    typedef int4 Elem;
    typedef thrust::device_vector<Elem> Offsets;
    int width;
    int height;
    Offsets offsets;

    MeshGrid(int width, int height){ 
        this->width = width;
        this->height = height;
        offsets.resize(width*height);
        thrust::counting_iterator<int> fst_index(0);
        thrust::counting_iterator<int> lst_index(width*height);
        thrust::transform(fst_index, lst_index, offsets.begin(), index_to_offset(width,height) );
    }
    Offsets::iterator begin(){ return offsets.begin(); }
    Offsets::iterator end(){ return offsets.end(); }
    
    /// a functor hashing indices to image-space coordinates
    struct index_to_offset {
        int n_rows, n_cols;
    
        __host__ __device__
        index_to_offset(int n_rows, int n_cols){
            this->n_rows = n_rows;
            this->n_cols = n_cols;
        }
        __host__ __device__
        int4 operator()(int index) {
            int4 offset;
            offset.y = index/n_rows;            // j
            offset.x = index-(n_rows*offset.y); // i
            offset.z = index;
            offset.w = -1; ///< invalid index
            return offset;
        }
    };
    
    struct GetPixelIndex : public thrust::unary_function<int, MeshGrid::Elem&>{
        __device__
        int operator()(MeshGrid::Elem& off){ return off.w; }
    };
};

//=============================================================================
} // namespace cudax
//=============================================================================

/// Ability to output Elem values to std::cout
inline std::ostream& operator<< (std::ostream& d, cudax::MeshGrid::Elem val) { 
    return d << val.x << " " << val.y << " " << val.z << " " << val.w;
}
