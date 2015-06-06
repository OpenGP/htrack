#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <limits>

using namespace std;

typedef float Scalar;
typedef Scalar Real;
typedef std::string String;
typedef unsigned int uint;
typedef int Integer;
typedef Eigen::Matrix4f Matrix4;
typedef Eigen::Matrix3f Matrix3;
typedef Eigen::Vector2f Vector2;
typedef Eigen::Vector3f Vector3;
typedef Eigen::Vector4f Vector4;
typedef Eigen::VectorXf VectorN;

/// Parameters vector
const int num_thetas = 29;
const int num_thetas_ignore = 9;
const int num_thetas_translation = 3;
const int num_thetas_rotation = 26;
const int num_thetas_rigid_motion = 6;
const int num_thetas_latent = 2;
const int num_thetas_thumb = 4;
const int num_thetas_fingers = 16;
const int num_thetas_pose = 20;
const int num_fingers = 5;

typedef Eigen::Matrix<Scalar, num_thetas, 1> Thetas;

/// More complex matrixes
typedef Eigen::Matrix<Scalar, 2, 3> Matrix_2x3;
typedef Eigen::Matrix<Scalar, 1, Eigen::Dynamic> Matrix_1xN;
typedef Eigen::Matrix<Scalar, 2, Eigen::Dynamic> Matrix_2xN;
typedef Eigen::Matrix<Scalar, 3, Eigen::Dynamic> Matrix_3xN;
typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 2> Matrix_Nx2;
typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Matrix_MxN;

typedef Eigen::Hyperplane<Scalar,3> Plane3;
typedef Eigen::ParametrizedLine<Scalar,3> Ray3;
typedef Eigen::AlignedBox3f BBox3;
typedef Eigen::Vector2i Vector2i;
typedef Eigen::AlignedBox2i BBox2i;
typedef Eigen::Quaternion<Real> Quaternion;

/// Nan for the default type
inline Real nan(){ return std::numeric_limits<Real>::quiet_NaN(); }
inline Real inf(){ return std::numeric_limits<Real>::max(); }

/// Linear system lhs*x=rhs
struct LinearSystem {
    Matrix_MxN lhs; // J^T*J
	VectorN rhs; // J^T*r
    LinearSystem() {}
    LinearSystem(int n) {
        lhs = Matrix_MxN::Zero(n,n);
        rhs = VectorN::Zero(n);
	}
};

