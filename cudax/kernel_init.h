#pragma once
#include "kernel.h"
#include "cudax/MeshGrid.h"
#include "cudax/Kinematic.h"
#include "cudax/PixelIndexer.h"

//=============================================================================
/// Constructor
//=============================================================================
void kernel_init(energy::fitting::Settings* _settings, int H_width, int H_height, int thetas_size, float H_focal_length_x, float H_focal_length_y, const float* H_inv_proj_matrix){
    cudax::settings = _settings;

    using namespace cudax;
    bool is_init = false;
    if(is_init){ std::cout << "!!!ERROR initialized cuda kernel twice" << std::endl; exit(0); }
    is_init = true;
            
    t = new cudax::CudaTimer();
    t->restart("Kernel Initialization");
        std::cout << "Init cuda: " << H_width << " " << H_height << std::endl;
        ///---
        meshgrid = new MeshGrid(H_width, H_height);
        ///---
        J = new thrust::device_vector<cudax::J_row>();
        e = new thrust::device_vector<float>();
        int upper_bound_num_constraints = 60000;        
        J->reserve(upper_bound_num_constraints);
        e->reserve(upper_bound_num_constraints);
        JtJ = new thrust::device_vector<float>(thetas_size*thetas_size);
        Jte = new thrust::device_vector<float>(thetas_size);
        ///---
        kinematic = new Kinematic();
        ///---
        silhouette_sensor = new thrust::device_vector<uchar>(H_width*H_height);
        ///--- 
        cudax::H_width = H_width;
        cudax::H_height = H_height;
        cudaMemcpyToSymbol(focal_length_x, &H_focal_length_x, sizeof(float));
        cudaMemcpyToSymbol(focal_length_y, &H_focal_length_y, sizeof(float));
        cudaMemcpyToSymbol(width, &H_width, sizeof(int));
        cudaMemcpyToSymbol(height, &H_height, sizeof(int));
        ///--- 
        camera_matrix = new KinectCamera(H_inv_proj_matrix);
        ///---
        indexes_memory = new thrust::device_vector<int2>(H_width*H_height, make_int2(-1,-1));
        ///---
        pixel_indexer = new PixelIndexer(*indexes_memory);

        ///---
        sensor_dtform_idxs = new thrust::device_vector<int>(H_width*H_height, -1);

        ///---
        cylinder_segments = new thrust::device_vector<float>(SEGMENT_VALUES*SEGMENT_JOINTS);
    t->display();
    // t->set_prefix(" + ");
    is_init = true;
}

//=============================================================================
/// Destructor
//=============================================================================
void kernel_cleanup(){
    std::cout << "kernel_cleanup()" << std::endl;
    using namespace cudax;    
    delete meshgrid;
    delete t;
    delete J;
    delete e;
    delete JtJ;
    delete Jte;
    delete kinematic;
    delete indexes_memory;
    delete sensor_dtform_idxs;
    delete cylinder_segments;
}

//=============================================================================
/// Checks
//=============================================================================
void kernel_memory_tests(){
    /// Make sure data alignment is not f'd when we pass data to cuda
    
    if( !(sizeof(glm::mat4x4) == (16*sizeof(float))) ){    
        printf("!!! Memory alignment error");
        exit(0);       
    }
    
    if( !(sizeof(cudax::J_row)==(sizeof(float)*NUM_THETAS)) ){
        printf("!!! Memory alignment error");
        exit(0);       
    }
    
    /// Make sure memory transfers are done correctly
    if(!(sizeof(cudax::J_row)==(sizeof(float)*NUM_THETAS))){
        printf("!!! Memory alignment error");
        exit(0);
    }
}
