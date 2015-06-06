#pragma once
//#define GLM_COMPILER 0
#define CUDA_VERSION 7000

#if __unix__
// GLM wants to redefine these functions, which causes 
// compilation errors in Linux. Undef'ing them WorksForMe...
#undef isnan
#undef isinf
#endif

#include "glm/glm.hpp"

/// To access raw pointer: glm::value_ptr(glm_mat4_variable)
// #include <glm/gtc/type_ptr.hpp>
