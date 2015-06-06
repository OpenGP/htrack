#pragma once
#include <vector>
#include <iostream>

#ifdef __CUDACC__
    #include "cudax/cuda_glm.h"
#else 
    #include "tracker/Types.h"
#endif

///--- Complexity below is to handle CPU/GPU types transparently
struct CustomJointInfo{
#ifdef __CUDACC__
    glm::mat4x4 mat;
    glm::vec3 axis;
#else
    float mat[16]; ///< joint name associated with this constraint
    float axis[3];  ///< axis to assemble jacobian column
#endif    
    int   type;      ///< how to assemble the column 
    int index;       ///< index of jacobian column
};

typedef std::vector<CustomJointInfo> JointTransformations;

/// Ability to output CustomJointInfo to std::cout (make sure transferred correctly)
inline std::ostream& operator<< (std::ostream& d, CustomJointInfo val) {
    d << "Jointinfo #" << val.index << std::endl;
    d << "type: " << val.type << std::endl;
    d << "matrix" << std::endl;
#ifndef __CUDACC__
    Eigen::Map<Matrix4> mat(val.mat);
#endif
    
    for (int row = 0; row<4; ++row) {
        for (int col = 0; col<4; ++col){
#ifdef __CUDACC__
            /// @note in column major, operator[] is not very intuitive!
            ///  this is because you first fetch the column in memory
            d << "  " << val.mat[col][row] << " ";
#else
            /// @note eigen operator() does it right instead
            d << "  " << mat(row,col) << " ";
#endif
        }
        d << std::endl;
    }
    d << "axis: \n  " << val.axis[0] << " " << val.axis[1] << " " << val.axis[2] << std::endl;
    return d;
}

#define CHAIN_MAX_LENGTH 15
struct ChainElement{ int data[CHAIN_MAX_LENGTH]; };
typedef std::vector<ChainElement> KinematicChain; ///< row major!

inline void clear_kinematic(KinematicChain& chain){
    for (int i = 0; i < chain.size(); ++i)
        for(int j=0; j<CHAIN_MAX_LENGTH; j++)
            chain[i].data[j] = -1;
    // memset(chain[i].data, -1, CHAIN_MAX_LENGTH); ///< wrong?
}

inline std::ostream& operator<< (std::ostream& d, ChainElement chain) {
    for(int j=0; j<CHAIN_MAX_LENGTH; j++)
        d << chain.data[j] << " ";
    return d;
}
inline std::ostream& operator<< (std::ostream& d, KinematicChain chains) {
    for (int i = 0; i < chains.size(); ++i)
        d << chains[i] << std::endl;   
    return d;    
}
