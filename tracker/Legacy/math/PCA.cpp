#include "PCA.h"
#include <iostream>

using namespace std;
using namespace Eigen;

//===========================================================================//

PCA::PCA()
{
	numParameters = 0;
	numFixedDof = 0;
	numDof = 0;

	sMin = 0.1f;
	sMax = 2.0f;

	numCor = 10;
	bufferMax = 500;
	iterEM = 2;

	inc = true;
}

//===========================================================================//

PCA::PCA(
	const vector<vector<float> > &data,
	size_t dof, size_t fDof, size_t fDofData)
{
	// initialize variables

	numParameters = 0;
	numFixedDof = 0;
	numDof = 0;

	sMin = 0.1f;
	sMax = 2.0f;

	numCor = 10;
	bufferMax = 500;
	iterEM = 2;

	inc = true;

	// perform pca on data

	size_t num = data.size();
	if(!num)
		return;

	size_t dim = data[0].size() - fDofData;
	if(!dof)
		dof = dim;

	numParameters = fDof + dim;
	numFixedDof = fDof;
	numDof = dof;

	// construct data matrix from input vector
	MatXf D(num, dim);
	for(size_t i = 0; i < num; ++i)
		for(size_t j = 0; j < dim; ++j)
			D(i, j) = data[i][fDofData + j];
	
	// compute mean-centered covariance matrix
	RowVectorXf M = D.colwise().mean();
	D.rowwise() -= M;
	MatXf C = D.adjoint() * D / num;

	// perform eigenvalue decomposition
	SelfAdjointEigenSolver<MatXf> E(C);

	// rearrange eigenvectors
	MatXf evs = E.eigenvectors();
	for(int i = 0; i < evs.cols() / 2; ++i)
		evs.col(i).swap(evs.col(evs.cols() - i - 1));

	// construct principal component matrix pcs
	pcs = MatXf::Zero(fDof + dim, fDof + dof);
	pcs.block(0, 0, fDof, fDof) << MatXf::Identity(fDof, fDof);
	pcs.block(fDof, fDof, dim, dof) << evs.block(0, 0, dim, dof);

	// store anchor matrix
	as = pcs;
	aaT = as * as.transpose();

	// construct eigenvalue vector eig
	eig = vector<float>(dim, 0);
	for(size_t i = 0; i < dim; ++i)
		eig[i] = E.eigenvalues()((dim - 1) - i);

	// construct data mean vector mid
	mid = vector<float>(fDof + dim, 0);
	for(size_t i = 0; i < fDof; ++i)
		mid[i] = 0;
	for(size_t i = fDof; i < fDof + dim; ++i)
		mid[i] = M(i - fDof);
}

//===========================================================================//

std::vector<float> PCA::toPC(const vector<float> &param, size_t newDof)
{
	if(!pcs.size())
		return param;

	size_t dim = numParameters;
	size_t dof = numFixedDof + (newDof ? newDof : numDof);

	VecXf p(dim);
	for(size_t i = 0; i < dim; ++i)
		p[i] = param[i] - mid[i];

	p = pcs.transpose() * p;

	vector<float> result(dof, 0);
	for(size_t i = 0; i < dof; ++i)
		result[i] = p[i];

	return result;
}

//===========================================================================//

std::vector<float> PCA::fromPC(const vector<float> &param, size_t newDof)
{
	if(!pcs.size())
		return param;

	size_t dim = numParameters;
	size_t dof = numFixedDof + numDof;

	VecXf p(dof);

	for(size_t i = 0; i < dof; ++i)
	{
		if(newDof && i >= numFixedDof + newDof)
			p[i] = 0;
		else
			p[i] = param[i];
	}

	p = pcs * p;

	vector<float> result(dim, 0);
	for(size_t i = 0; i < dim; ++i)
		result[i] = p[i] + mid[i];

	return result;
}

//===========================================================================//

MatXf PCA::getPCs(size_t dof)
{
	if(!pcs.size())
		return MatXf::Identity(numParameters, numParameters);

	if(dof && dof < (size_t)numDof)
		return MatXf(pcs.block(0, 0, numParameters, numFixedDof + dof));

	return pcs;
}

//===========================================================================//

VecXf PCA::adapt(
	const vector<float> &param)
{
#if 0
	if(!as.size())
		return VecXf();

	// compute projected residual sample
	VecXf p(numParameters);
	VecXf m(numParameters);
	for(int i = 0; i < numParameters; ++i)
	{
		p[i] = param[i];
		m[i] = mid[i];
	}
	VecXf sample = (p - m) - aaT * (p - m);

	// check validity of sample
	if(isValid(sample))
	{
		// in-place covariance computation
		bool inPlaceCov = true;
		if(inPlaceCov)
		{
			// update sample buffer
			VecXf sample_out = VecXf::Zero(sample.size());
			buffer.push_back(sample);
			if(buffer.size() > (size_t)bufferMax)
			{
				sample_out = buffer.front();
				buffer.pop_front();
			}

			int d = numParameters - numFixedDof;

			// not enough samples
			if(buffer.size() < (size_t)d)
				return VecXf();

			// initialize
			if(!cs.size() || buffer.size() < (size_t)bufferMax)
			{
				// mean
				s_mean = VecXf::Zero(d);
				for(size_t i = 0; i < buffer.size(); ++i)
				{
					VecXf s_i = buffer[i].block(numFixedDof,0,d,1);
					s_mean += s_i;
				}
				s_mean /= buffer.size();

				// covariance matrix
				C_ = MatXf::Zero(d,d);
				for(size_t i = 0; i < buffer.size(); ++i)
				{
					VecXf s_i = buffer[i].block(numFixedDof,0,d,1);
					C_ += (s_i - s_mean) * (s_i - s_mean).transpose();
				}
				MatXf C=C_;

				// perform eigenvalue decomposition
				SelfAdjointEigenSolver<MatXf> E(C);

				// rearrange eigenvectors
				MatXf evs = E.eigenvectors();
				for(int i = 0; i < evs.cols() / 2; ++i)
					evs.col(i).swap(evs.col(evs.cols() - i - 1));

				// construct corrective principal component matrix cs
				cs = MatXf::Zero(numParameters, numCor);
				cs.block(numFixedDof, 0, numParameters - numFixedDof, numCor)
					<< evs.block(0, 0, numParameters - numFixedDof, numCor);

				// update anchors in pc-matrix, increment dofs
				if(inc)
				{
					pcs = MatXf(as.rows(), as.cols() + cs.cols());
					pcs.block(0,0,as.rows(),as.cols()) = as;

					numDof += numCor;
					inc = false;
				}

				// update correctives in pc-matrix
				pcs.block(0,as.cols(),cs.rows(),cs.cols()) = cs;
			}

			// update
			if(buffer.size() == (size_t)bufferMax)
			{
				VecXf s_in = sample.block(numFixedDof,0,d,1);
				VecXf s_out = sample_out.block(numFixedDof,0,d,1);

				VecXf s_mean_prev = s_mean;
				float N = (float)buffer.size();
				s_mean += s_in / N - s_out / N;

				for(int i = 0; i < d; ++i)
					for(int j = 0; j < d; ++j)
						C_(i,j) += s_mean_prev[i] * s_mean_prev[j]
								 - s_mean[i] * s_mean[j]
								 - s_out[i] * s_out[j] / N
								 + s_in[i] * s_in[j] / N;
				MatXf C=C_;

				// perform eigenvalue decomposition
				SelfAdjointEigenSolver<MatXf> E(C);

				// rearrange eigenvectors
				MatXf evs = E.eigenvectors();
				for(int i = 0; i < evs.cols() / 2; ++i)
					evs.col(i).swap(evs.col(evs.cols() - i - 1));

				// construct corrective principal component matrix cs
				cs = MatXf::Zero(numParameters, numCor);
				cs.block(numFixedDof, 0, numParameters - numFixedDof, numCor)
					<< evs.block(0, 0, numParameters - numFixedDof, numCor);

				// update correctives in pc-matrix
				pcs.block(0,as.cols(),cs.rows(),cs.cols()) = cs;
			}

			return sample;
		}


		//-- standard svd pca instead of incremental covariance pca

		// update sample buffer
		buffer.push_back(sample);
		if(buffer.size() > (size_t)bufferMax)
			buffer.pop_front();

		// initialize correctives if necessary/possible
		if(!cs.size() && buffer.size() >= (size_t)numCor)
		{
			// construct correctives from buffer without pose dofs
			cs = MatXf(numParameters - numFixedDof, numCor);
			for(int i = 0; i < numParameters - numFixedDof; ++i)
				for(int j = 0; j < numCor; ++j)
					cs(i,j) = buffer[j][numFixedDof + i];

			// orthogonalize matrix wit QR decomposition
			ColPivHouseholderQR<MatXf> qr(cs);
			cs = MatXf(qr.matrixQ()).block(
					0,0, numParameters - numFixedDof, numCor);

			// add pose dofs to corrective matrix
			MatXf tmp = cs;
			cs = MatXf(numParameters, numCor);
			for(int i = 0; i < numParameters; ++i)
			{
				for(int j = 0; j < numCor; ++j)
				{
					if(i < numFixedDof)
						cs(i,j) = 0;
					else
						cs(i,j) = tmp(i - numFixedDof, j);
				}
			}

			// join anchors and correctives in pc-matrix
			pcs = MatXf(as.rows(), as.cols() + cs.cols());
			pcs.block(0,0,as.rows(),as.cols()) = as;
			pcs.block(0,as.cols(),cs.rows(),cs.cols()) = cs;

			numDof += numCor;
		}
		// update correctives if possible
		else if(cs.size())
		{
			// construct buffer matrix S
			MatXf S(buffer.size(), numParameters - numFixedDof);
			for(size_t j = 0; j < buffer.size(); ++j)
				for(int i = 0; i < numParameters - numFixedDof; ++i)
					S(j,i) = buffer[j][i + numFixedDof];
			S /= sqrt((float)buffer.size());

			// perform singular value decomposition
			Eigen::JacobiSVD<MatXf> svd(S, ComputeThinV);

			// construct corrective principal component matrix cs
			cs = MatXf::Zero(numParameters, numCor);
			cs.block(numFixedDof, 0, numParameters-numFixedDof, numCor)
				<< svd.matrixV().block(0, 0, numParameters-numFixedDof, numCor) * -1.0f;

			// update correctives in pc-matrix
			pcs.block(0,as.cols(),cs.rows(),cs.cols()) = cs;
		}
	}

	return sample;
#else
    std::cout << "!!!!!!!! TODO FIXME PCA" << std::endl;
    VecXf retval;
    return retval;
#endif
}

//===========================================================================//

void PCA::reset()
{
	if(!pcs.size())
		return;

	pcs = as;
	cs = MatXf();
	buffer.clear();
	numDof = as.cols() - numFixedDof;
}

//===========================================================================//

void PCA::expand()
{
	if(!pcs.size())
		return;

	as = pcs;
	aaT = as * as.transpose();
	cs = MatXf();
	buffer.clear();
	numDof = as.cols() - numFixedDof;
}

//===========================================================================//

