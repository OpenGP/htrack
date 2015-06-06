#pragma once
#include <iostream>
#include <iomanip>
#include <fstream>

//== NAMESPACE ================================================================
namespace cudax {
//=============================================================================

// http://devblogs.nvidia.com/parallelforall/how-implement-performance-metrics-cuda-cc/
struct CudaTimer{
    cudaEvent_t _start, _stop;
    std::string message;
    std::string prefix;
    bool event_on_destroy;
    /// Same as create + restart
    CudaTimer(const std::string& message){
        cudaEventCreate(&_start);
        cudaEventCreate(&_stop);        
        this->message = message;
        cudaEventRecord(_start);
        event_on_destroy = true;
        std::cout << std::setprecision(3);
        prefix = "Executed ";
    }
    CudaTimer(){
        prefix = "Executed ";
        event_on_destroy = false;
        cudaEventCreate(&_start);
        cudaEventCreate(&_stop);        
    }
    ~CudaTimer(){
        if(event_on_destroy)
            display();
        cudaEventDestroy(_start);
        cudaEventDestroy(_stop);
    }
    void set_prefix(const std::string& prefix){
        this->prefix = prefix;
    }
    void restart(const std::string& message){
        this->message = message;
        cudaEventRecord(_start);        
    }
    void display(){
        static std::ofstream out("cuda.log");
        cudaEventRecord(_stop);       
        cudaEventSynchronize(_stop);
        float milliseconds = 0;
        cudaEventElapsedTime(&milliseconds, _start, _stop);
        
        std::cout << prefix << "["<< message <<"] in [" << milliseconds << "ms]" << std::endl;
    }
	float elapsed(){
        cudaEventRecord(_stop);       
        cudaEventSynchronize(_stop);
        float milliseconds = 0;
        cudaEventElapsedTime(&milliseconds, _start, _stop);
		return milliseconds;
	}
};

//=============================================================================
} // namespace cudax
//=============================================================================

#define CUDA_TIMED_BLOCK(obj, blockName) for ( struct { int i; cudax::CudaTimer timer; } obj = { 0, cudax::CudaTimer(blockName) }; obj.i < 1; ++obj.i)
