#include "MathUtils.h"

#include <Eigen/Dense>
#include "../util/Util.h"
#include "../util/openmp_helpers.h"
using namespace Eigen;

//===========================================================================//

Vec3f toEuler(const Mat3f &m, int e1, int e2, int e3)
{
	return m.eulerAngles(e1,e2,e3);
}

//===========================================================================//

Vec3f toEuler(const Mat4f &m, int e1, int e2, int e3)
{
	Mat3f m_ = m.block(0,0,3,3);
	return toEuler(m_, e1, e2, e3);
}

//===========================================================================//

Mat3f fromEuler(const Vec3f &v, int e1, int e2, int e3)
{
	return Mat3f(
		AngleAxisf(v[0], Vec3f::Unit(e1)) *
		AngleAxisf(v[1], Vec3f::Unit(e2)) *
		AngleAxisf(v[2], Vec3f::Unit(e3)) );
}

//===========================================================================//

Mat3f fromEuler(float a, float b, float c, int e1, int e2, int e3)
{
	return fromEuler(Vec3f(a, b, c), e1, e2, e3);
}

//===========================================================================//

MatXf pinv(const MatXf &sym)
{
	return sym.llt().solve(MatXf::Identity(sym.rows(), sym.rows()));
}

//===========================================================================//

MatXf transp(const MatXf &m)
{
	size_t r = m.rows();
	size_t c = m.cols();

	MatXf mT(c, r);

    openmp::setNumThreads(openmp::NUM_THREADS);
//    #pragma omp parallel for
	for(size_t i = 0; i < r; ++i)
		for(size_t j = 0; j < c; ++j)
			mT(j,i) = m(i,j);

	return mT;
}

//===========================================================================//

MatXf addpar(const MatXf &a, const MatXf &b)
{
	size_t r = a.rows();
	size_t c = a.cols();

	assert(b.rows() == r && b.cols() == c);

	MatXf m(r, c);

    openmp::setNumThreads(openmp::NUM_THREADS);
    #pragma omp parallel for collapse(2)
	for(size_t i = 0; i < r; ++i)
		for(size_t j = 0; j < c; ++j)
			m(i,j) = a(i,j) + b(i,j);

	return m;
}

//===========================================================================//

MatXf maxvol(const MatXf &A)
{
	std::vector<int>_;
	return maxvol(A,_);
}

//===========================================================================//

MatXf maxvol(const MatXf &A, std::vector<int> &indices)
{
	// matrix dimensions
	int n = A.rows();
	int m = A.cols();

	if(m >= n)
	{
		for(int i = 0; i < m; ++i)
			indices.push_back(i);

		return A;
	}

	// LU decomposition of A with pivoting
	// for initialization of submatrix Ap
	Eigen::FullPivLU<MatXf> lu(A);

	// initialize permuted A_ and B
	MatXf A_ = lu.permutationP() * A;
	MatXf Ap = A_.block(0,0,m,m);
	MatXf B  = A_ * Ap.inverse();

	// initialize index vector with permutation indices
	indices = std::vector<int>(
		lu.permutationP().indices().data(),
		lu.permutationP().indices().data() + n);

	// iterative matrix re-ordering
	for(int i = m; i < n; ++i)
	{
		// find index j of maximum absolute coefficient in row i
		int j = i;
		float Bij = B.row(i).cwiseAbs().maxCoeff(&j);

		if(Bij <= 1.0f)
			break;

		// swap rows i and j in the current solution
		A_.row(i).swap(A_.row(j));
		std::swap(indices[i], indices[j]);

		// update B
		const VecXf &ej = VecXf::Unit(n, j);
		const VecXf &ei = VecXf::Unit(n, i);
		const RowVectorXf &ejT = VecXf::Unit(m, j);

		B = B - (B.col(j) - ej + ei) * (B.row(i) - ejT) / Bij;
	}

	return A_.block(0,0,m,m);
}

//===========================================================================//

