#include "util/gl_wrapper.h" ///< for cuda_gl_interop
#include <cuda_gl_interop.h>
#include <thrust/host_vector.h>
#include <thrust/copy.h>

#include "cudax/kernel.h"
#include "cudax/CudaTimer.h"
#include "cudax/helper_cuda.h" ///< SDK error checking
#include "cudax/CublasHelper.h"
#include "cudax/CudaHelper.h"
#include "cudax/OpenCVOutputBuffer.h"
#include "cudax/KinectCamera.h"
#include "cudax/kernel_init.h"
#include "cudax/kernel_upload.h"
#include "cudax/kernel_debug.h"
#include "cudax/PixelIndexer.h"

#include "cudax/functors/IsSilhouette.h"
#include "cudax/functors/IsSilhouetteBoundary.h"
#include "cudax/functors/ImageHighlighter.h"
#include "cudax/functors/IsTopLeft3x3Block.h"
#include "cudax/functors/IsMatchingDepth.h"
#include "cudax/functors/ComputeJacobianRowSilho.h"
#include "cudax/functors/ComputeJacobianRowDepth.h"
#include "cudax/functors/ComputeJacobianRowPush.h"
#include "cudax/functors/ComputeJacobianRowExtraClosedForm.h"

#include <iostream>



using namespace cudax;

struct absolute_value : public thrust::unary_function <float, float > {
	__host__ __device__
		float operator()(float x) const {
		return (x>=0) ? x : -x;
	}
};

void kernel_bind()
{
    // CUDA_TIMED_SCOPE(timer,"cuda bind textures") /// microseconds
    if(cudax::render_color)   CHECK_CUDA(cudaBindTextureToArray(color_tex, cudax::render_color));
    if(cudax::render_points)  CHECK_CUDA(cudaBindTextureToArray(extra_tex, cudax::render_points));
    if(cudax::render_normals) CHECK_CUDA(cudaBindTextureToArray(norms_tex, cudax::render_normals));
    if(cudax::sensor_depth)   CHECK_CUDA(cudaBindTextureToArray(depth_tex, cudax::sensor_depth));
    if(cudax::sensor_normals) CHECK_CUDA(cudaBindTextureToArray(sensor_normals_tex, cudax::sensor_normals));
}

void kernel_unbind(){
    // CUDA_TIMED_SCOPE(timer,"Unbind cuda textures") /// microseconds
    if(cudax::render_color)   CHECK_CUDA(cudaUnbindTexture(color_tex));
    if(cudax::render_points)  CHECK_CUDA(cudaUnbindTexture(extra_tex));
    if(cudax::render_normals) CHECK_CUDA(cudaUnbindTexture(norms_tex));
    if(cudax::sensor_depth)   CHECK_CUDA(cudaUnbindTexture(depth_tex));
    if(cudax::sensor_normals) CHECK_CUDA(cudaUnbindTexture(sensor_normals_tex));

    cudax::render_color=NULL;
    cudax::render_points=NULL;
    cudax::render_normals=NULL;
    cudax::sensor_depth=NULL;
    cudax::sensor_normals=NULL;
}

void kernel(float* eigen_JtJ, float* eigen_Jte, float & push_error, float & pull_error, bool eval_metric, bool reweight)
{    
    //cudaDeviceSynchronize();
    
    // CUDA_TIMED_BLOCK(timer,"indexing constraints")
    pixel_indexer->exec();
    // pixel_indexer->print_constraints();
    
    int n_silho = pixel_indexer->num_silho_constraints();
    int n_depth = pixel_indexer->num_depth_constraints();
    int n_pull = pixel_indexer->num_extra_pull_constraints();
    int n_push = pixel_indexer->num_extra_push_constraints();
      
    //cudaDeviceSynchronize();
      
    // CUDA_TIMED_BLOCK(timer,"memory resize + zero (J+e)")
    {
        int n_total = n_silho + n_depth + n_pull + n_push;
		
        J->resize(n_total);
        e->resize(n_total);
        /// @todo consider if possible do this better
        const J_row zeros = {};
        thrust::fill(J->begin(),J->end(),zeros);
        thrust::fill(e->begin(),e->end(),0.0f);
        
        if(n_total==0)
            return;
    }
        
    //cudaDeviceSynchronize();
    ///--- Pointer math time!
    J_row* J_silho = thrust::raw_pointer_cast(J->data());
    J_row* J_depth = J_silho + n_silho;
    J_row* J_push = J_depth + n_depth;
    J_row* J_pull = J_push + n_push;    
    
    float* e_silho = thrust::raw_pointer_cast(e->data());
    float* e_depth = e_silho + n_silho;
    float* e_push = e_depth + n_depth;
    float* e_pull = e_push + n_push;
    
    ComputeJacobianRowSilho functor_silho(J_silho, e_silho);
    ComputeJacobianRowDepth functor_depth(J_depth, e_depth);
    ComputeJacobianRowPush functor_push(J_push, e_push);
    
    ComputeJacobianRowExtraClosedForm functor_pull(J_pull, e_pull, reweight);

    if(store_corresps){
        thrust::fill(debug_closedform_correspondences->begin(),
                     debug_closedform_correspondences->end(), -111);
        functor_pull.store_data(thrust::raw_pointer_cast(
            debug_closedform_correspondences->data()));
    }
    
    // CUDA_TIMED_BLOCK(timer,"Assemble Jacobian")
    {
        if( settings->silho_enable )
            //CUDA_TIMED_BLOCK(timer,"assemble silho jacobian")
                thrust::for_each(meshgrid->begin(), meshgrid->end(), functor_silho);
        if( settings->depth_enable )
            //CUDA_TIMED_BLOCK(timer,"assemble depth jacobian")
                thrust::for_each(meshgrid->begin(), meshgrid->end(), functor_depth);
        if( settings->fit2D_enable )
            // CUDA_TIMED_BLOCK(timer,"assemble push jacobian")
                thrust::for_each(meshgrid->begin(), meshgrid->end(), functor_push);
        if( settings->fit3D_enable )
            // CUDA_TIMED_BLOCK(timer,"assemble pull jacobian")
                thrust::for_each(meshgrid->begin(), meshgrid->end(), functor_pull);
    }
    
#if 0
    kernel_simplify_jacobian(J->size());
    cudaDeviceSynchronize();
#endif
    
#if 0
    float e_sum = 0;
    thrust::host_vector<float> tmp_e = *e;
    for(int i=0; i<n_pull; ++i){
        float err = tmp_e[n_silho + n_depth + n_push + i];
        e_sum += err;
    }
    printf("pull residual sum: %f\n", e_sum);
#endif

    //cudaDeviceSynchronize();
    
    // CUDA_TIMED_BLOCK(timer, "Jt*J and Jt*e + CPU Transfer")
    {
        float ret = 0;
        CublasHelper::outer_product_J(*J, *JtJ, J->size(), NUM_THETAS);
        CublasHelper::vector_product_J(*J, *e, *Jte, J->size(), NUM_THETAS);
        CublasHelper::dot_product_e(*e, ret, e->size()); ///< error measure
        thrust::copy(JtJ->begin(), JtJ->end(), eigen_JtJ);
        thrust::copy(Jte->begin(), Jte->end(), eigen_Jte);
    }
    //cudaDeviceSynchronize();    
   
    /// Only need evaluate metric on the last iteration
    if (eval_metric) {
		push_error = (float) pixel_indexer->num_silho_overlap() / (float) pixel_indexer->num_silho_union();

		thrust::device_vector<float> f(n_pull);
		thrust::transform(e->begin() + n_silho + n_depth + n_push, e->begin() + n_silho + n_depth + n_push + n_pull, f.begin(), absolute_value());
		pull_error = thrust::reduce(f.begin(), f.end());
		pull_error = pull_error / n_pull * 3;
	}		

    return;
}
