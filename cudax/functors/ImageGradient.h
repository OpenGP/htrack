#include "cudax/kernel.h"
#include "cudax/functors/IsSilhouette.h"
#include "cudax/cuda_glm.h"

//== NAMESPACE ================================================================
namespace cudax {
//=============================================================================

///--- anonymous so it's not exposed outside
namespace { __constant__ float gradient_kernel[3][3]; }

struct ImageGradient{
    IsRenderedSilhouette is_silhouette; ///< @todo this might be changed with other functors if needed
    
    ImageGradient(){
        ///--- Initialization of constant memory
        float sobel[3][3] = {{-1, 0, 1}, {-2, 0, 2}, {-1, 0, 1}};
        cudaMemcpyToSymbol(gradient_kernel, sobel, 9*sizeof(float));
    }

    __device__
    glm::vec2 operator()(const MeshGrid::Elem& off){
        glm::vec2 ret;
        
        // printf("kernel gradient: \n");
        #pragma unroll
        for(int i = -1; i <= 1; ++i){
            #pragma unroll 
            for(int j = -1; j <= 1; ++j){
                int x = off.x+i;
                int y = off.y+j;
                float value = (float) is_silhouette(x,y);
                ret[0] += gradient_kernel[i+1][j+1] * value;
                ret[1] += gradient_kernel[j+1][i+1] * value;
                // printf("%d ", is_silhouette(x,y));                
            }
            // printf("\n");
        }
        if(glm::dot(ret,ret) < 0.1f){
            return glm::vec2(0,0);
        } else {
            ret = glm::normalize(ret);
        }
        
#define ENABLE_MATCH_CPU_GRADIENT
#ifdef ENABLE_MATCH_CPU_GRADIENT
        float tmp = -ret[0];
        ret[0] = -ret[1];
        ret[1] = tmp;
#endif
        
        return ret;
    }
};

//=============================================================================
} // namespace cudax
//=============================================================================
