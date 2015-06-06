#pragma once
#include <thrust/device_vector.h>

//== NAMESPACE ================================================================
namespace cudax {
//=============================================================================

///--- Deals with debug memory copy&mapping
class OpenCVOutputBuffer{
    typedef unsigned char uchar;
    uchar* host_mem;
    thrust::device_vector<uchar> dev_mem;
    int _width, _height;
public:
    OpenCVOutputBuffer(uchar* host_mem, int width, int height) : 
        host_mem(host_mem),
        dev_mem(host_mem, host_mem+(width*height)),
        _width(width), _height(height){}
    ~OpenCVOutputBuffer(){
        thrust::copy(dev_mem.begin(), dev_mem.end(), host_mem);                
    }
    uchar* get(){ return dev_mem.data().get(); }  
    int width(){ return _width; }
    int height(){ return _height; }
};

//=============================================================================
} // namespace cudax
//=============================================================================
