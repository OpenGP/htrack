#pragma once
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cuda_runtime.h>
#include <cublas_v2.h>
#include "thrust/device_vector.h"
#include <cassert>
#include "util/singleton.h"
#include "externs.h"

//== NAMESPACE ================================================================
namespace cudax {
//=============================================================================


class CublasHelper{
    SINGLETON(CublasHelper)

public:
    cublasHandle_t _handle;
    
public:
    static cublasHandle_t handle(){ return instance()._handle; }
    static void init(){
        std::cout << "Starting up CUBLAS" << std::endl;
        cublasStatus_t status = cublasCreate(&(instance()._handle));
        if (status != CUBLAS_STATUS_SUCCESS){
            fprintf(stderr, "!!!! CUBLAS initialization error\n");
            exit(EXIT_FAILURE);
        }
    }
    static void cleanup(){
        std::cout << "Shutting down CUBLAS" << std::endl;        
        cublasStatus_t status = cublasDestroy(instance()._handle);
        if (status != CUBLAS_STATUS_SUCCESS) {
            fprintf(stderr, "!!!! CUBLAS shutdown error\n");
            exit(EXIT_FAILURE);
        }   
    }


//-----------------------------------------------------------------------------


public:
    static void outer_product_J(const Jacobian& J, ///< input
                              thrust::device_vector<float>& JtJ, ///< output
                              int num_constr, ///< #rows of J
                              int num_pars) ///< #cols of J
    {
        assert(J.size()==(num_constr)); ///< check jacobian rows
        assert(JtJ.size()==(num_pars*num_pars));
        outer_product((float*)thrust::raw_pointer_cast(J.data()),
                      thrust::raw_pointer_cast(&JtJ[0]),
                      num_constr,
                      num_pars);
    }
        
public:
    static void vector_product_J(const Jacobian& J, ///< input
                               const thrust::device_vector<float>& e, ///< input
                               thrust::device_vector<float>& Jte, ///< output
                               int num_constr, ///< #rows of J and e
                               int num_pars) ///< #cols of J
    {
        assert(J.size()==(num_constr)); ///< check jacobian rows
        assert(e.size()==(num_constr));
        assert(Jte.size()==(num_pars));
        vector_product((float*)thrust::raw_pointer_cast(J.data()),
                       thrust::raw_pointer_cast(&e[0]),
                       thrust::raw_pointer_cast(&Jte[0]),
                       num_constr,
                       num_pars);
    }
    
public:
    static void dot_product_e(const thrust::device_vector<float>& e, ///< input
                              float& ete, ///< output
                              int num_constr) ///< # elements of vector e
    {
        assert(e.size()==(num_constr));
        dot_product(thrust::raw_pointer_cast(&e[0]),
                    thrust::raw_pointer_cast(&e[0]),
                    &ete, num_constr);
    }
    
    
//-----------------------------------------------------------------------------


public:
    static void outer_product(const thrust::device_vector<float>& J, ///< input
                              thrust::device_vector<float>& JtJ, ///< output
                              int num_constr, ///< #rows of J
                              int num_pars) ///< #cols of J
    {
        assert(J.size()==(num_constr*num_pars));
        assert(JtJ.size()==(num_pars*num_pars));
        outer_product((float*)thrust::raw_pointer_cast(J.data()),
                      (float*)thrust::raw_pointer_cast(&JtJ[0]),
                      num_constr,
                      num_pars);
    }
        
public:
    static void vector_product(const thrust::device_vector<float>& J, ///< input
                               const thrust::device_vector<float>& e, ///< input
                               thrust::device_vector<float>& Jte, ///< output
                               int num_constr, ///< #rows of J and e
                               int num_pars) ///< #cols of J
    {
        assert(J.size()==(num_constr*num_pars));
        assert(e.size()==(num_constr));
        assert(Jte.size()==(num_pars));
        vector_product((float*)thrust::raw_pointer_cast(J.data()),
                       (float*)thrust::raw_pointer_cast(&e[0]),
                       (float*)thrust::raw_pointer_cast(&Jte[0]),
                       num_constr,
                       num_pars);
    }
        
    
//-----------------------------------------------------------------------------
    

private:
    /// Computes J^t*J and stores in pre-allocated matrix JtJ
    static void outer_product(const float* J, float* JtJ, int num_constr, int num_pars /*num_thetas*/){
        // C = \beta C + \alpha op(A) op(B);
        //
        // "ld" refers to the leading dimension of the matrix, which in the case of column-major 
        // storage is the number of rows of the allocated matrix (even if only a submatrix of 
        // it is being used).
        //
        const float alpha = 1.0;
        const float beta = 0.0;

#define ROW_MAJOR_INPUT
#ifdef ROW_MAJOR_INPUT
        // Cublas expects column major matrices, but our input is row major.
        // Therefore we let cublas treat our row major input as column major
        // and compute: J_colmajor * J_colmajor^T == J_rowmajor^T * J_rowmajor
        cublasStatus_t status = cublasSgemm(CublasHelper::handle(), 
                                            CUBLAS_OP_N, // J_rowmajor^T == J_colmajor
                                            CUBLAS_OP_T, // J_rowmajor   == J_colmajor^T
                                            num_pars, num_pars, num_constr, 
                                            &alpha,
                                            J, // A 
                                            num_pars, // ld_A
                                            J, // B, 
                                            num_pars, // ld_B
                                            &beta,
                                            JtJ, // C
                                            num_pars // ld_C
                                            );
#else
        // column major input matrix
        cublasStatus_t status = cublasSgemm(CublasHelper::handle(), 
                                            CUBLAS_OP_T, //J^t
                                            CUBLAS_OP_N, //J
                                            num_pars, num_pars, num_constr, 
                                            &alpha,
                                            J, // A 
                                            num_constr, // ld_A
                                            J, // B, 
                                            num_constr, // ld_B
                                            &beta,
                                            JtJ, // C
                                            num_pars // ld_C
                                            );
#endif

        if(status != CUBLAS_STATUS_SUCCESS){
            fprintf(stderr, "!!!! CUBLAS matrix multiplication (constr: %d pars: %d) failure!\n", num_constr, num_pars);
            // fprintf(stderr, "!!!! CUBLAS matrix multiplication failure!\n");
            exit(EXIT_FAILURE);
        }  
    }

private:
    /// Computes J^t*e and stores in pre-allocated matrix Jte
    static void vector_product(
            const float* J,
            const float* e,
            float* Jte,
            int num_constr,
            int num_pars /*num_thetas*/)
    {
        // y = \alpha op(A) x + \beta y
        const float alpha = 1.0;
        const float beta = 0.0;
        const int inc1 = 1;

        // Cublas expects column major matrices, but our input is row major.
        // Therefore we let cublas treat our row major input as column major
        // and compute: J_colmajor * e == J_rowmajor^T * e
#if 1
        cublasStatus_t status = cublasSgemv(CublasHelper::handle(), 
                                            CUBLAS_OP_N, // J_rowmajor^T == J_colmajor
                                            num_pars, num_constr, 
                                            &alpha, // 1
                                            J, // A 
                                            num_pars, // ld_A
                                            e, // x
                                            inc1, // stride of x
                                            &beta, // 0
                                            Jte, // y
                                            inc1 // stride of y
                                            );
#else
        cublasStatus_t status = cublasSgemm(CublasHelper::handle(), 
                                            CUBLAS_OP_N, // J_rowmajor^T == J_colmajor
                                            CUBLAS_OP_T, // e_rowmajor   == e_colmajor^T
                                            num_pars, 1, num_constr, 
                                            &alpha,
                                            J, // A 
                                            num_pars, // ld_A
                                            e, // B, 
                                            inc1, // ld_B
                                            &beta,
                                            Jte, // C
                                            inc1 // ld_C
                                            );
#endif

        if(status != CUBLAS_STATUS_SUCCESS){
            fprintf(stderr, "!!!! CUBLAS matrix-vector multiplication (constr: %d pars: %d) failure!\n", num_constr, num_pars);
            exit(EXIT_FAILURE);
        }  
    }
    
private:
    /// Computes u^t*v and stores in referenced variable utv
    static void dot_product(
            const float* u,
            const float* v,
            float* utv,
            int dim)
    {
        /*
           cublasStatus_t cublasSdot (cublasHandle_t handle, int n,
                           const float           *x, int incx,
                           const float           *y, int incy,
                           float           *result)
        */
        
        cublasStatus_t status = cublasSdot(CublasHelper::handle(),
                                           dim,
                                           u, 1,
                                           v, 1,
                                           utv);
        
        if(status != CUBLAS_STATUS_SUCCESS){
            fprintf(stderr, "!!!! CUBLAS vector-vector multiplication (d: %d) failure!\n", dim);
            exit(EXIT_FAILURE);
        }  
    }
};

//=============================================================================
} // namespace cudax
//=============================================================================
