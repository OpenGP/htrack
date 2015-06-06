// #include "cudax/kernel.h"
#include "cudax/OpenCVOutputBuffer.h"
#include "cudax/MeshGrid.h"

//== NAMESPACE ================================================================
namespace cudax {
//=============================================================================

/// http://docs.thrust.googlecode.com/hg/structthrust_1_1unary__function.html
struct CopyOver : public thrust::unary_function<void, int4>{
    enum MODE{_color, _idxs} mode;
    uchar* out;
    float width, height;
    CopyOver(uchar* out, int width, int height):out(out),width(width),height(height){}
    
    __device__
    void operator()(int4 off){
        switch(mode){
            case _color: out[off.z] = tex2D(color_tex, off.x, off.y); break; ///< Simply copies in to out 
            case _idxs: out[off.z] = off.w*255/counter; break; ///< see correct indexes 
        }

        // out[off.z] = tex2D(extra_tex, off.x, off.y).x; ///< test 
        // out[off.z] = tex2D(depth_tex, off.x, off.y).x; ///< test 
        // out[off.z] = (off.x/width)*255; ///< print x coords
        // out[off.z] = (off.y/height)*255; ///< print y coords        
    }    
    
    static void copyover(MeshGrid* meshgrid, uchar* output, CopyOver::MODE mode){
        OpenCVOutputBuffer _output(output, meshgrid->width, meshgrid->height);
        CopyOver functor(_output.get(), meshgrid->width, meshgrid->height);
        functor.mode = mode;
        thrust::for_each(meshgrid->begin(), meshgrid->end(), functor);
    }
    static void color(MeshGrid* meshgrid, uchar* output){copyover(meshgrid, output, _idxs);}
    static void idxs(MeshGrid* meshgrid, uchar* output){copyover(meshgrid, output, _color);}
};

//=============================================================================
} // namespace cudax
//=============================================================================
