#include <thrust/version.h>
#include <thrust/host_vector.h>
#include <thrust/device_vector.h> 
#include <thrust/sort.h>
#include <iostream>
#include "util/tictoc.h"
#include "cudax/CudaHelper.h"

int main(void)
{
    int major = THRUST_MAJOR_VERSION;
    int minor = THRUST_MINOR_VERSION;
    std::cout << "Thrust version: " << major << "." << minor << std::endl;
    thrust::device_vector<int> d_vec;
    thrust::host_vector<int> h_vec(1 << 24); 
    
    TICTOC_BLOCK(t_gen,"Generate randoms + h2d transfer")
    {
        thrust::generate(h_vec.begin(), h_vec.end(), rand);
        d_vec = h_vec; ///<  
    }
    
    TICTOC_BLOCK(t_sort,"GPU sort")
    {
        thrust::sort(d_vec.begin(), d_vec.end());           
        cudaDeviceSynchronize(); ///< otherwise time is 0!!
    }
    
    TICTOC_BLOCK(t_back,"d2h transfer")
    {
        thrust::copy(d_vec.begin(), d_vec.end(), h_vec.begin()); 
    }
    return 0;
}
