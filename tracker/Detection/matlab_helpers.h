#pragma once
#include "tracker/Types.h"

namespace matlab{

/// see matlab randn
static inline Real randn( const double mean, const double stddev ){
    double u1 = ( static_cast<double> ( std::rand() ) + 1 ) / ( (double)RAND_MAX + 1 );
    double u2 = ( static_cast<double> ( std::rand() ) + 1 ) / ( (double)RAND_MAX + 1 );
    assert( -2 * std::log( u1 ) >= 0 );
    double t1 = std::sqrt( -2 * std::log( u1 ) ) * cos( 2 * M_PI * u2 );
    return mean + stddev * t1;
}
static inline Real rand(){
    /// @todo check this one actually gives [0..1]
    return ( static_cast<double> ( std::rand() ) ) / ( (double)RAND_MAX );
}


inline Matrix_MxN rand(int rows, int cols){
    Matrix_MxN m(rows,cols);
    for(int i=0; i<m.rows(); i++)
        for(int j=0; j<m.cols(); j++)
            m(i,j) = rand();
    return m;
}


template <typename T>
inline size_t nnz(const T & A) {
    size_t M = A.rows();
    size_t N = A.cols();
    size_t k = 0;
    for (size_t i = 0; i < M; i++) {
        for (size_t j = 0; j < N; j++) {
            if (A(i, j) != 0.0)
                k++;
        }
    }
    return k;
}


inline size_t sub2ind(size_t R, size_t C, Vector2s s) {
    size_t r = s(0);
    size_t c = s(1);
    return c * R + r;
}


inline Vector2s ind2sub(size_t R, size_t C, size_t i) {
    size_t c = i / R;
    size_t r = i % R;
    Vector2s s = Vector2s(r, c);
    return s;
}


template <typename T>
inline int sign(T val) {
    return (T(0) < val) - (val < T(0));
}


template<typename T>
inline void find(const T & A, VectorNs & R, VectorNs & C) {
    size_t num_non_zeros = nnz<T>(A);
    R = VectorNs::Zero(num_non_zeros);
    C = VectorNs::Zero(num_non_zeros);
    size_t M = A.rows();
    size_t N = A.cols();
    size_t k = 0;
    for (size_t r = 0; r < M; r++) {
        for (size_t c = 0; c < N; c++) {
            if (A(r, c) > 0) {
                R(k) = r;
                C(k) = c;
                k++;
            }
        }
    }
}


template<typename T>
inline void find(const T & A, VectorNs & I) {
    size_t num_non_zeros = nnz(A);
    I = VectorNs::Zero(num_non_zeros);
    size_t M = A.rows();
    size_t N = A.cols();
    size_t k = 0;
    for (size_t r = 0; r < M; r++) {
        for (size_t c = 0; c < N; c++) {
            size_t i = sub2ind(M, N, Vector2s(r, c));
            if (A(r, c) > 0) {
                I(k) = i;
                k++;
            }
        }
    }
}


template<typename T>
inline void find(const T & A, VectorNs & R, VectorNs & C, VectorN & V) {
    size_t num_non_zeros = nnz<T>(A);
    R = VectorNs::Zero(num_non_zeros);
    C = VectorNs::Zero(num_non_zeros);
    V = VectorNs::Zero(num_non_zeros);
    size_t M = A.rows();
    size_t N = A.cols();
    size_t k = 0;
    for (size_t r = 0; r < M; r++) {
        for (size_t c = 0; c < N; c++) {
            if (A(r, c) > 0) {
                R(k) = r;
                C(k) = c;
                V(k) = (float) A(r, c);
                k++;
            }
        }
    }
}


template<typename T>
inline float mean(const T & a) {
    float sum = a.sum();
    return sum / a.size();
}


template<typename T>
inline float stddev(const T & a) {
    return sqrt((a.array() - mean(a)).array().pow(2).sum() / (a.size() - 1));
}


} ///< matlab::
