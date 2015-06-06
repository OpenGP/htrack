/// @warning NEVER include this on the C++ side, only externals
#pragma once

#ifndef __CUDACC__
    #error you cannot compile c++ of this, only cudax/externs.h can be included
#endif

///--- system
#include <iostream>
#include <cuda_runtime.h>
#include <thrust/device_vector.h>
#include "cudax/cuda_glm.h"
#include "cudax/externs.h" ///< only thing that is exposed to C++
#include "tracker/Energy/Fitting/Settings.h"

//== NAMESPACE ================================================================
namespace cudax {
//=============================================================================

//=============================================================================
/// Forward declarations
//=============================================================================
class Kinematic;
class KinectCamera;
class MeshGrid;
struct CudaTimer;
class PixelIndexer;

//=============================================================================
/// Settings for fitting energy
//=============================================================================
energy::fitting::Settings* settings;

//=============================================================================
/// Kernel constants
//=============================================================================
__constant__ float focal_length_x;
__constant__ float focal_length_y;
__constant__ int width;
__constant__ int height;
int H_width;
int H_height;

//=============================================================================
/// Pixel type
//=============================================================================
namespace PixelType{ 
enum TYPE{INVALID=-1, 
          CONSTRAINT_DEPTH=0, 
          CONSTRAINT_SILHO=1, 
          CONSTRAINT_EXTRA_PULL=2,
          CONSTRAINT_EXTRA_PUSH=3,
          RENDER_SILHOUETTE=4, 
          SENSOR_SILHOUETTE=5,
          OVERLAP_SILHOUETTE=6,
          UNION_SILHOUETTE=7,
          SIZE=8}; }

///--- Texture accessors
#define SENSOR_DEPTH(off) tex2D(depth_tex, off.x, (height-1-off.y)).x //< Depth is from kinect sensor, sunny side up!
#define SENSOR_NORMALS(off) tex2D(sensor_normals_tex, off.x, (height-1-off.y)) //< Depth is from kinect sensor, sunny side up!
#define RENDER_DEPTH(off) tex2D(extra_tex, off.x, off.y).z

///--- Texture types
typedef texture<uchar, 2, cudaReadModeElementType> GrayscaleTexture2D;
typedef texture<float4, 2, cudaReadModeElementType> Float4Texture2D;
typedef texture<ushort1, 2, cudaReadModeElementType> DepthTexture2D; /// GL_R16UI

//=============================================================================
/// Global memory
//=============================================================================
///--- These are the arrays mapped to the textures below
cudaArray* render_color = NULL;
cudaArray* render_points = NULL;
cudaArray* render_normals = NULL;
cudaArray* sensor_depth = NULL;
cudaArray* sensor_normals = NULL;

///--- Textures containing the OpenGL input data (must be declared in global scope)
GrayscaleTexture2D color_tex; ///< joint_id of rendered model
Float4Texture2D    extra_tex; ///< xyz of rendered model
Float4Texture2D    norms_tex; ///< normals of rendered model
DepthTexture2D     depth_tex; ///< texture with depth from sensor
Float4Texture2D    sensor_normals_tex;
CudaTimer* t = NULL;
MeshGrid* meshgrid = NULL;

///--- Transferred from tracking context
thrust::device_vector<uchar>* silhouette_sensor = NULL;
thrust::device_vector<int>* sensor_dtform_idxs = NULL;
    
Jacobian* J = NULL; ///< semi-preallocated memory to store jacobian
thrust::device_vector<float>* e = NULL; ///< effectors (same # columns as J)
thrust::device_vector<float>* JtJ = NULL; ///< preallocated memory NUM_THETAS^2
thrust::device_vector<float>* Jte = NULL; ///< preallocated memory NUM_THETAS
uchar* opencv_image = NULL;

thrust::device_vector<int2>* indexes_memory = NULL; ///< pixel to constraint type + index
PixelIndexer* pixel_indexer = NULL;


KinectCamera* camera_matrix = NULL;
Kinematic* kinematic = NULL;
    
#define SEGMENT_VALUES 44
#define SEGMENT_JOINTS 17
thrust::device_vector<float>* cylinder_segments = NULL;
thrust::device_vector<float>* debug_closedform_correspondences = NULL;
bool store_corresps = false;

//=============================================================================
} // namespace cudax
//=============================================================================







