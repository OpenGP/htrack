#pragma once
#include "cudax/kernel.h"
#include "cudax/functors/IsSilhouetteBoundary.h"
#include "cudax/functors/IsSilhouette.h"
#include "cudax/OpenCVOutputBuffer.h"

//== NAMESPACE ================================================================
namespace cudax {
//=============================================================================

template <class MeshGridElemBoolFunctor>
struct ImageHighlighter{
    MeshGridElemBoolFunctor condition;
    uchar* out;
    ImageHighlighter(OpenCVOutputBuffer& opencv){
        out = opencv.get();        
    }
    
    __device__
    void operator()(MeshGrid::Elem& off){
        if(condition(off))
            out[off.z] = 255;
        else
            out[off.z] = 0;
    }
};

/// Specializations
typedef ImageHighlighter<IsSilhouetteBoundary> SilhouetteBoundaryHighlighter;
typedef ImageHighlighter<IsRenderedSilhouette> SilhouetteHighlighter;

//=============================================================================
} // namespace cudax
//=============================================================================
