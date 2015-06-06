#pragma once
#include "kernel.h"
#include <thrust/device_vector.h>
#include "cudax/functors/IsMatchingDepth.h"
#include "cudax/functors/IsSilhouetteBoundary.h"
#include "cudax/functors/IsSilhouette.h"
#include "cudax/MeshGrid.h"

//== NAMESPACE ================================================================
namespace cudax {
//=============================================================================

struct PixelIndexer : public thrust::unary_function<void, MeshGrid::Elem&>{
    const int INVALID;
    thrust::device_vector<int> counters_memory; 
    thrust::device_vector<int> render_indexes_memory; ///< row index of a pixel in the render cloud
    thrust::device_vector<int> sensor_indexes_memory; ///< row index of a pixel in the sensor cloud   
    int*  counters;       ///< raw devive pointer!! 
    int*  render_indexes; ///< raw devive pointer!!
    int*  sensor_indexes; ///< raw devive pointer!!
    int2* cnstr_indexes;  ///< raw devive pointer!!
    
public:    
    /// @see https://github.com/thrust/thrust/blob/master/examples/arbitrary_transformation.cu
    PixelIndexer(thrust::device_vector<int2>& indexes)
        : INVALID(-1),
        counters_memory(PixelType::SIZE, 0 /*init*/),
        render_indexes_memory(H_width*H_height, PixelIndexer::INVALID),
        sensor_indexes_memory(H_width*H_height, PixelIndexer::INVALID)
    {
        this->counters = thrust::raw_pointer_cast(&counters_memory[0]);
        this->render_indexes = thrust::raw_pointer_cast(&render_indexes_memory[0]);
        this->sensor_indexes = thrust::raw_pointer_cast(&sensor_indexes_memory[0]);       
        this->cnstr_indexes  = thrust::raw_pointer_cast(&indexes[0]);
    }
    
    void clear(){
        thrust::fill(counters_memory.begin(), counters_memory.end(), 0);
        /// @todo these could be done during operator() for efficiency
        thrust::fill(render_indexes_memory.begin(), render_indexes_memory.end(), PixelIndexer::INVALID);
        thrust::fill(sensor_indexes_memory.begin(), sensor_indexes_memory.end(), PixelIndexer::INVALID);
    }
    
    int num_silho_constraints(){ return counters_memory[PixelType::CONSTRAINT_SILHO]; }
    int num_depth_constraints(){ return counters_memory[PixelType::CONSTRAINT_DEPTH]; }
    int num_extra_pull_constraints(){ return 3*counters_memory[PixelType::CONSTRAINT_EXTRA_PULL]; }
    int num_extra_push_constraints(){ return 2*counters_memory[PixelType::CONSTRAINT_EXTRA_PUSH]; }
    
    int num_silho_sensor(){ return counters_memory[PixelType::SENSOR_SILHOUETTE]; }
    int num_silho_render(){ return counters_memory[PixelType::RENDER_SILHOUETTE]; }
    int num_silho_overlap(){ return counters_memory[PixelType::OVERLAP_SILHOUETTE]; }
    int num_silho_union(){ return counters_memory[PixelType::UNION_SILHOUETTE]; }
    
    void print_constraints(){
        // printf(" #silho: %d\n", num_silho_constraints());
        // printf(" #depth: %d\n", num_depth_constraints());
        printf(" #extra_pull: %d\n", num_extra_pull_constraints());
        // printf(" #extra_push: %d\n", num_extra_push_constraints());
        // printf(" #rendered: %d\n", num_silho_render());
    }
    
    __device__ int type(const MeshGrid::Elem& off){ return cnstr_indexes[off.z].x; }
    __device__ int constraint_index(const MeshGrid::Elem& off){ return cnstr_indexes[off.z].y; }
    __device__ int render_cloud_index(const MeshGrid::Elem& off){ return render_indexes[off.z]; }
    __device__ int sensor_cloud_index(const MeshGrid::Elem& off){ return sensor_indexes[off.z]; }
    
    void exec(){
        clear();
        Functor functor(counters, render_indexes, sensor_indexes, cnstr_indexes);
        thrust::for_each(meshgrid->begin(), meshgrid->end(), functor);
    }
    
    struct IsExtraPullConstraint{
        IsSensorSilhouette is_sensor_silhouette;
        IsExtraPullConstraint() : is_sensor_silhouette(*silhouette_sensor){}

        __device__
        bool operator()(const int4& off){
#ifndef DISCARD_BAD_SENSOR
            return is_sensor_silhouette(off);
#else
            if(!is_sensor_silhouette(off)) return false;
            return true;
            float4 sensor_normal = SENSOR_NORMALS(off);
            // printf("sensor_normal %f %f %f\n", sensor_normal.x, sensor_normal.y, sensor_normal.z);
            return (-sensor_normal.z > .05); ///< minus: normal point toward camera
#endif 
        }
    };
    
    struct IsExtraPushConstraint{
        IsRenderedSilhouette is_r_silho;
        IsSensorSilhouette is_s_silho;
        IsExtraPushConstraint() : is_s_silho(*silhouette_sensor){}

        __device__
        bool operator()(const int4& off){
            return is_r_silho(off) && !is_s_silho(off);
        }
    };

    ///--- @note unfortunately functors cannot contain device_vector members, so we need to move 
    ///          operator() in a nested functor class
    struct Functor{
        // typedef IsSensorSilhouette IsExtraPullConstraint; ///< TEMP        
        // typedef IsRenderedSilhouette IsExtraPushConstraint; ///< TEMP
        IsDepthConstraint       is_depth_constraint;
        IsSilhouetteBoundary    is_silhouette_boundary;
        IsRenderedSilhouette    is_rendered_silhouette;
        IsSensorSilhouette      is_sensor_silhouette;
        IsExtraPullConstraint   is_extra_pull_constraint;
        IsExtraPushConstraint   is_extra_push_constraint;
        
        int*  counters;       ///< global memory holding pixels in 
        int*  render_indexes; ///< row associated to pixel in the render cloud
        int*  sensor_indexes; ///< row associated to pixel in the sensor cloud
        int2* cnstr_indexes;
        
        bool silho_enabled;
        bool depth_enabled;
        bool extra_pull_enabled;
        bool extra_push_enabled;
        int  extra_skip_every;
        
        Functor(int* counters, int* render_indexes, int* sensor_indexes, int2* cnstr_indexes) : 
            is_depth_constraint(*silhouette_sensor),
            is_sensor_silhouette(*silhouette_sensor)
        {
            this->counters = counters;
            this->render_indexes = render_indexes;
            this->sensor_indexes = sensor_indexes;       
            this->cnstr_indexes  = cnstr_indexes;
            
            silho_enabled = settings->silho_enable;
            depth_enabled = settings->depth_enable;
            extra_push_enabled = settings->fit2D_enable;
            extra_pull_enabled = settings->fit3D_enable;
            extra_skip_every = settings->fit3D_stepsize;
        }

        __device__
        void operator()(MeshGrid::Elem& off){
            ///--- Invalidate constraints
            cnstr_indexes[off.z] = make_int2( PixelType::INVALID, PixelType::INVALID );

            ///--- Update counters for tracking failure detection
			{   
				bool is_rendered = is_rendered_silhouette(off);
				bool is_sensor = is_sensor_silhouette(off);
                if( is_rendered) atomicAdd(&counters[PixelType::RENDER_SILHOUETTE],1);
                if( is_sensor)	atomicAdd(&counters[PixelType::SENSOR_SILHOUETTE],1);
                if( is_rendered && is_sensor ) atomicAdd(&counters[PixelType::OVERLAP_SILHOUETTE],1);
                if( is_rendered || is_sensor ) atomicAdd(&counters[PixelType::UNION_SILHOUETTE],1);
            }
            
            ///--- Mark constraints
            if( silho_enabled && is_silhouette_boundary(off) ){
                cnstr_indexes[off.z] = make_int2(PixelType::CONSTRAINT_SILHO, atomicAdd(&counters[PixelType::CONSTRAINT_SILHO],1));
                return;
            }

            if( depth_enabled && is_depth_constraint(off) && /*for safety only*/ !is_silhouette_boundary(off) ){
                cnstr_indexes[off.z] = make_int2(PixelType::CONSTRAINT_DEPTH, atomicAdd(&counters[PixelType::CONSTRAINT_DEPTH],1));
                return;
            }
            
            ///--- Anything not taken care by depth and silhouette arrives here
            if( extra_pull_enabled && is_extra_pull_constraint(off) ){
                cnstr_indexes[off.z] = make_int2(PixelType::CONSTRAINT_EXTRA_PULL, atomicAdd(&counters[PixelType::CONSTRAINT_EXTRA_PULL],1));    
                return;
            }
            if( extra_push_enabled && is_extra_push_constraint(off) ){
                cnstr_indexes[off.z] = make_int2(PixelType::CONSTRAINT_EXTRA_PUSH, atomicAdd(&counters[PixelType::CONSTRAINT_EXTRA_PUSH],1));
                return;
            }
        }
    };
};

//=============================================================================
} // namespace cudax
//=============================================================================
