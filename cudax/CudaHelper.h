/// @warning this cannot contain any c++11!!
#pragma once
#include <cuda_runtime.h>
#include "util/gl_wrapper.h"
#include <cuda_gl_interop.h>
#include "util/singleton.h"
#include "cudax/helper_cuda.h"

//== NAMESPACE ================================================================
namespace cudax {
//=============================================================================

class CudaHelper{
    SINGLETON(CudaHelper)
    
public:
    /// @note to be done ***after*** OpenGL for optimal OpenGL/CUDA performance
    static void init(){
        int devID = gpuGetMaxGflopsDeviceId();
        cudaError status = cudaGLSetGLDevice(devID);
        if(status!=cudaSuccess){
            std::cout << "Could not get OpenGL compliant device... exiting" << std::endl;
            exit(0);
        }
    }
    static void cleanup(){
        std::cout << "Shutting down CUDA device" << std::endl;
        cudaDeviceReset();
    }
    
public:
    /// To debug calls to: cudaBindTextureToArray
    static void check_array(cudaArray* array){
        struct cudaChannelFormatDesc desc;
        checkCudaErrors(cudaGetChannelDesc(&desc, array));
        printf("CUDA Array channel descriptor, bits per component:\n");
        printf("X %d Y %d Z %d W %d, kind %d\n",
               desc.x,desc.y,desc.z,desc.w,desc.f);
        printf("Possible values for channel format kind: i %d, u%d, f%d:\n",
               cudaChannelFormatKindSigned, cudaChannelFormatKindUnsigned,
               cudaChannelFormatKindFloat);        
    }
};

//=============================================================================
} // namespace cudax
//=============================================================================

