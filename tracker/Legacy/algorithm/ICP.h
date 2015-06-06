#pragma once

#include "../math/Inverse_kinematics.h"
#include "../math/MathUtils.h"
#include "../geometry/Cylinders.h"

class ICP
{
	public:

		ICP() {}

		void align(
            std::vector<Vec3f> &points, Cylinders &model,
			int iterRigid = 2, int iterSkeletal = 3, int iterIK = 5,
			float thresholdRigid = -1, float thresholdSkeletal = -1);

		void alignRigid(
            std::vector<Vec3f> &points, Cylinders &model,
			int iterations, float threshold = -1);

		void alignSkeletal(
            std::vector<Vec3f> &points, Cylinders &model,
			int iterations, int iterationsIK, float threshold = -1);

		float correspond(
            std::vector<Vec3f> &points, Cylinders &model);

		Mat4f registration(
			const std::vector<Vec3f> &src,
			const std::vector<Vec3f> &dst);

		void prediction(Skeleton *skeleton);

		void adaptPCA(Skeleton *skeleton, int iterIK = 5);

		void reset() { effectors.clear(); }

	public:

		void alignWithWristband(
			std::vector<Vec3f> &points,
			std::vector<Vec3f> &wristband,
            Cylinders &cylinders,
			int iterRigid = 2, int iterSkeletal = 3, int iterIK = 5,
			float thresholdRigid = -1, float thresholdSkeletal = -1);

		float correspondPart(
			std::vector<Vec3f> &points,
			Cylinders &cylinders,
			const std::string &jointName);

        void rigidICP(Cylinders &model,
            const std::vector<Vec3f> &points,
            int iterations);

	public:

		std::vector<Vec3f> effectors;
		std::vector<float> previous;
		Inverse_kinematics ik;

};

