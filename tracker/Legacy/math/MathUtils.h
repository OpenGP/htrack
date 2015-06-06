#pragma once

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <map>

// 3d vector types
typedef Eigen::Vector3f Vec3f;
typedef Eigen::Vector3d Vec3d;

// 4d vector types
typedef Eigen::Vector4f Vec4f;
typedef Eigen::Vector4d Vec4d;

// dynamic vector types
typedef Eigen::VectorXf VecXf;
typedef Eigen::VectorXd VecXd;

// 3x3 matrix types
typedef Eigen::Matrix<float, 3, 3, Eigen::ColMajor> Mat3f;
typedef Eigen::Matrix<double, 3, 3, Eigen::ColMajor> Mat3d;

// 4x4 matrix types
typedef Eigen::Matrix<float, 4, 4, Eigen::ColMajor> Mat4f;
typedef Eigen::Matrix<double, 4, 4, Eigen::ColMajor> Mat4d;

// dynamic matrix types
typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor> MatXf;
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor> MatXd;

// affine transformation types
typedef Eigen::Transform<float, 3, Eigen::Affine, Eigen::ColMajor> Transform3f;
typedef Eigen::Transform<double, 3, Eigen::Affine, Eigen::ColMajor> Transform3d;

// aligned stl vector for eigen3 (substitute with c++11 typedef alias when gcc supports it)
template<typename T>
struct aligned_vector {
	typedef std::vector<T, Eigen::aligned_allocator<T> > type;
};

// aligned stl map for eigen3 (substitute with c++11 typedef alias when gcc supports it)
template<typename K, typename V>
struct aligned_map {
	typedef std::map<K, V, std::less<K>,
			Eigen::aligned_allocator<std::pair<const K, V> > > type;
};

// euler angle conversion functions
Vec3f toEuler(const Mat3f &m, int e1, int e2, int e3);
Vec3f toEuler(const Mat4f &m, int e1, int e2, int e3);
Mat3f fromEuler(const Vec3f &v, int e1, int e2, int e3);
Mat3f fromEuler(float a, float b, float c, int e1, int e2, int e3);

// matrix functions
MatXf pinv(const MatXf &sym);
MatXf transp(const MatXf &m);
MatXf addpar(const MatXf &a, const MatXf &b);
MatXf maxvol(const MatXf &m);
MatXf maxvol(const MatXf &m, std::vector<int> &indices);

