#pragma once

#include <vector>
#include <deque>
#include "MathUtils.h"

// pca usage:
// - inputs are database, desired pc space dimension and fixed dof.
// - fixed dof are the parts of the kinematic model that are unaffected by the
//   pc space mapping because they are not contained in the database. they must
//   be defined as the first elements of the joint angle / pc space vectors.
//   for me, these are the six global translation / rotation parameters.
// - use fromPC and toPC to convert between joint angle space and pc space.

class PCA
{
	public:

		PCA();

		PCA(const std::vector<std::vector<float> > &data,
			size_t dof = 0, size_t fixedDof = 0, size_t fixedDofData = 0);

		std::vector<float> toPC(const std::vector<float> &param, size_t newDof = 0);
		std::vector<float> fromPC(const std::vector<float> &param, size_t newDof = 0);

		MatXf getPCs(size_t dof);

		VecXf adapt(const std::vector<float> &param);
		void reset();
		void expand();

		bool isValid(VecXf &sample) {
			float s = sample.norm();
			return (s > sMin && s < sMax);
		}

	public:

		MatXf pcs;

		std::vector<float> eig;
		std::vector<float> mid;

		MatXf as;
		MatXf cs;
		MatXf aaT;

		std::deque<VecXf> buffer;

		int numParameters;
		int numFixedDof;
		int numDof;

		int numCor;
		int bufferMax;
		int iterEM;

		float sMin;
		float sMax;

		VecXf s_mean;
		MatXf M_;
		MatXf C_;
		bool inc;
};
