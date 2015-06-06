#pragma once
#include <cuda_runtime.h>
#include <thrust/device_vector.h>
#include <thrust/host_vector.h>
#include "cudax/cuda_glm.h"
#include <vector>

namespace energy{namespace fitting{struct Settings;}}
struct CustomJointInfo;
struct ChainElement;
typedef unsigned char uchar;

//== NAMESPACE ================================================================
namespace cudax {
//=============================================================================

///--- Jacobian type
#define NUM_THETAS 29 ///< hand parameter size (size of jacobian row)
struct J_row{ float data[NUM_THETAS]; }; ///< Row of Jacobian
typedef thrust::device_vector<J_row> Jacobian; ///< ROW major!!!

///--- Externally defined resources
extern cudaArray* render_color;
extern cudaArray* render_points;
extern cudaArray* render_normals;
extern cudaArray* sensor_depth;
extern cudaArray* sensor_normals;

//=============================================================================
} // namespace cudax
//=============================================================================

extern "C"
void kernel_init(energy::fitting::Settings* settings,
                 int _width,
                 int _height,
                 int thetas_size,
                 float H_focal_length_x,
                 float H_focal_length_y,
                 const float *H_inv_proj_matrix);

extern "C" void kernel_upload_kinematic(const std::vector<CustomJointInfo>& jointinfos, const std::vector<ChainElement>& H_kinchains);
extern "C" void kernel_upload_cylinders(const float* H_cylinders);
extern "C" void kernel_upload_sensor_data(uchar* H_silhouette_sensor);
extern "C" void kernel_upload_dtform_idxs(int* H_dtform_idxs);

extern "C" void kernel_bind();
extern "C" void kernel(float* eigen_JtJ, float* eigen_Jte, float & push_error, float & pull_error, bool eval_metric, bool reweight);
extern "C" void kernel_unbind();

extern "C"
void kernel_copy_extra_corresp(thrust::host_vector<float4>& H_queries, 
                               thrust::host_vector<float4>& H_target, 
                               thrust::host_vector<int>& H_idxs);

extern "C"
void kernel_store_closedform_corresp(int num);

extern "C"
void kernel_copy_closedform_corresp(thrust::host_vector<float> &H_closedform_corrs);

extern "C"
void kernel_cleanup();

extern "C"
void kernel_memory_tests();

extern "C"
void kernel_constraint_type_image(uchar *, int, int);

extern "C"
void kernel_simplify_jacobian();

