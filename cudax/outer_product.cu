#include "thrust/device_vector.h"
#include "thrust/host_vector.h"
#include "thrust/copy.h"
#include "cudax/Timer.h"
#include "cudax/CudaTimer.h"

#include "CublasHelper.h"

using namespace cudax;

extern "C" void outer_product(float* input, float* output, int rows, int cols){
    thrust::device_vector<float> J(input, input+(rows*cols));    
    thrust::device_vector<float> JtJ(cols*cols);
    CublasHelper::outer_product(J, JtJ, rows, cols);
    thrust::copy(JtJ.begin(), JtJ.end(), output);
}

extern "C" void vector_product(float* J_in, float* e_in, float* Jte_out, int rows, int cols){
    thrust::device_vector<float> J(J_in, J_in+(rows*cols));    
    thrust::device_vector<float> e(e_in, e_in+(rows));    
    thrust::device_vector<float> Jte(cols);
    CublasHelper::vector_product(J, e, Jte, rows, cols);
    thrust::copy(Jte.begin(), Jte.end(), Jte_out);
}

//--

thrust::device_vector<float> _test_J;
thrust::device_vector<float> _test_JtJ;

extern "C" void outer_product_init(float* input, int rows, int cols){
    _test_J = thrust::device_vector<float>(input, input+(rows*cols));    
    _test_JtJ = thrust::device_vector<float>(cols*cols);
}

extern "C" void outer_product_compute(int rows, int cols){
    CublasHelper::outer_product(_test_J, _test_JtJ, rows, cols);
}

extern "C" void outer_product_copy(float* output){
    thrust::copy(_test_JtJ.begin(), _test_JtJ.end(), output);
}

//--

thrust::device_vector<float> _test_J2;
thrust::device_vector<float> _test_e;
thrust::device_vector<float> _test_Jte;

extern "C" void vector_product_init(float* J_in, float* e_in, int rows, int cols){
    _test_J2 = thrust::device_vector<float>(J_in, J_in+(rows*cols));    
    _test_e = thrust::device_vector<float>(e_in, e_in+(rows));    
    _test_Jte = thrust::device_vector<float>(cols);
}

extern "C" void vector_product_compute(int rows, int cols){
    CublasHelper::vector_product(_test_J2, _test_e, _test_Jte, rows, cols);
}

extern "C" void vector_product_copy(float* Jte_out){
    thrust::copy(_test_Jte.begin(), _test_Jte.end(), Jte_out);
}


