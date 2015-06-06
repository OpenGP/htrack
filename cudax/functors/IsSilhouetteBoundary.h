#pragma once
#include "cudax/kernel.h"

//== NAMESPACE ================================================================
namespace cudax {
//=============================================================================

struct IsSilhouetteBoundary{
    __device__
    bool operator()(const int4 off){
        uchar p0 = tex2D(color_tex, off.x, off.y);

        /// Don't bother processing outside pixels
        if(p0==255) {
            return false;
        }
        
        /// Inside pixels, keep only the ones that see outside        
        for(int i= -1; i<=1; i++){
            for(int j=-1; j<=1; j++){
                if( tex2D(color_tex,off.x+i,off.y+j)==255 )
                    return true;
            }
        }
        
        /// This one didn't see any outside
        return false;
    }
};

//=============================================================================
} // namespace cudax
//=============================================================================
