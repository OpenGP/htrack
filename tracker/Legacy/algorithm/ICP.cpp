#include "ICP.h"

#include <cassert>
#include <iostream>

//===========================================================================//

void ICP::align(
    std::vector<Vec3f> &points, Cylinders &model,
	int iterationsRigid, int iterationsSkeletal, int iterationsIK,
	float thresholdRigid, float thresholdSkeletal)
{
	reset();

	if(points.size() < 3)
		return;

	prediction(model.getSkeleton());

	alignRigid(points, model, iterationsRigid, thresholdRigid);
	alignSkeletal(points, model, iterationsSkeletal, iterationsIK, thresholdSkeletal);

	model.update();
}

//===========================================================================//

void ICP::alignRigid(
    std::vector<Vec3f> &points, Cylinders &model,
	int iterations, float threshold)
{
    if(iterations < 1)
        return;

	float e = threshold + 1;
	float pe = 0;

	if(effectors.empty())
		e = correspond(points, model);

	if(points.size() < 3)
		return;

	model.transform(registration(effectors, points));

	int i = 1;

	while(i++ < iterations && fabs(e - pe) > threshold)
	{
		pe = e;
		e = correspond(points, model);

		if(points.size() < 3)
			return;

		model.transform(registration(effectors, points));
	}
}

//===========================================================================//

void ICP::alignSkeletal(
    std::vector<Vec3f> &points, Cylinders &model,
	int iterations, int iterationsIK, float threshold)
{
    if(iterations < 1)
        return;

	float e = threshold + 1;
	float pe = 0;

	if(effectors.empty())
		e = correspond(points, model);

	if(points.size() < 3)
		return;

	for(int j = 0; j < iterationsIK; ++j)
		ik.solve(*model.getSkeleton());

	int i = 1;

	while(i++ < iterations && fabs(e - pe) > threshold)
	{
		pe = e;
		e = correspond(points, model);

		if(points.size() < 3)
			return;

		for(int j = 0; j < iterationsIK; ++j)
			ik.solve(*model.getSkeleton());
	}
}

//===========================================================================//

float ICP::correspond(
    std::vector<Vec3f> &points, Cylinders &model)
{
	float e = 0;
	model.update();
	effectors.resize(points.size());
	std::vector<std::string> names(points.size());
	std::vector<bool> remove(points.size(), true);

#pragma omp parallel for
	for(size_t i = 0; i < points.size(); ++i)
	{
		Vec3f p, n;
		std::string name;

		float d = model.nearest(points[i], p, n, name);

		if(!isnan(d) && d < 50) remove[i] = false;
		else e += d*d;

		effectors[i] = p;
		names[i] = name; // string not thread safe in clang
	}

	Skeleton *skeleton = model.getSkeleton();
	skeleton->clearEffectors();

	for(size_t i = points.size(); i-->0;)
	{
		if(remove[i])
		{
			points.erase(points.begin() + i);
			effectors.erase(effectors.begin() + i);
			names.erase(names.begin() + i);
		}
		else
		{
			Joint *j = skeleton->getJoint(names[i]);
			Effector *e = new Effector(j, effectors[i], points[i]);
			skeleton->addEffector(e);
		}
	}

	return e;
}

//===========================================================================//

void ICP::adaptPCA(Skeleton *skeleton, int iterIK)
{
	Mapping &m = skeleton->getMapping();

	if(m.getPCAEnabled() && effectors.size())
	{
		// refine posture with all dofs
		m.setPCAEnabled(false);
		for(int i = 0; i < iterIK; ++i)
			ik.solve(*skeleton);

		// adapt pca with resulting posture
		m.setPCAEnabled(true);
		m.adaptPCA(skeleton->getCurrentParameters());
	}
}

//===========================================================================//

void ICP::prediction(Skeleton *skeleton)
{
	const std::vector<float> &current = skeleton->getCurrentParameters();
	if(previous.empty())
		previous = current;

	std::vector<float> dt(current.size());
	for(size_t i = 0; i < dt.size(); ++i)
		dt[i] = current[i] - previous[i];

	ik.setPredictionUpdate(dt);
	previous = current;
}

//===========================================================================//

Mat4f ICP::registration(
		const std::vector<Vec3f>& _src,
		const std::vector<Vec3f>& _dst)
{
	assert(_src.size() == _dst.size());
	const int n = _src.size();
	assert(n>2);



	// compute barycenters
	Vec3f scog(0.0,0.0,0.0), dcog(0.0,0.0,0.0);
	for (int i=0; i<n; ++i)
	{
		scog += _src[i];
		dcog += _dst[i];
	}
	scog /= (float) n;
	dcog /= (float) n;



	// build matrix
	Mat4d M;
	{
		double  xx(0.0), xy(0.0), xz(0.0), yx(0.0), yy(0.0), yz(0.0), zx(0.0), zy(0.0), zz(0.0);
		Vec3f   sp, dp;

		for (int i=0; i<n; ++i)
		{
			sp = _src[i]; sp -= scog;
			dp = _dst[i]; dp -= dcog;
			xx += sp[0] * dp[0];
			xy += sp[0] * dp[1];
			xz += sp[0] * dp[2];
			yx += sp[1] * dp[0];
			yy += sp[1] * dp[1];
			yz += sp[1] * dp[2];
			zx += sp[2] * dp[0];
			zy += sp[2] * dp[1];
			zz += sp[2] * dp[2];
		}

		M(0,0) =  xx + yy + zz;
		M(1,1) =  xx - yy - zz;
		M(2,2) = -xx + yy - zz;
		M(3,3) = -xx - yy + zz;
		M(1,0) = M(0,1) = yz - zy;
		M(2,0) = M(0,2) = zx - xz;
		M(2,1) = M(1,2) = xy + yx;
		M(3,0) = M(0,3) = xy - yx;
		M(3,1) = M(1,3) = zx + xz;
		M(3,2) = M(2,3) = yz + zy;
	}


	// symmetric eigendecomposition
	Mat4d   V = Mat4d::Identity();
	unsigned int iter(50);
	{
		int     i, j, k;
		double  theta, t, c, s, ss, g, h, tau, tM;

		while (--iter)
		{
			// find largest off-diagonal element
			i=0; j=1; ss=fabs(M(0,1));
			if ( (s=fabs(M(0,2))) > ss) { ss=s; i=0; j=2; }
			if ( (s=fabs(M(0,3))) > ss) { ss=s; i=0; j=3; }
			if ( (s=fabs(M(1,2))) > ss) { ss=s; i=1; j=2; }
			if ( (s=fabs(M(1,3))) > ss) { ss=s; i=1; j=3; }
			if ( (s=fabs(M(2,3))) > ss) { ss=s; i=2; j=3; }

			// converged?
			if (ss < 1e-10) break;

			// compute Jacobi rotation
			theta = 0.5 * (M(j,j) - M(i,i)) / M(i,j);
			t     = (theta<0.0 ? -1.0 : 1.0) / (fabs(theta) + sqrt(1.0+theta*theta));
			c     = 1.0 / sqrt(1.0 + t*t);
			s     = t*c;
			tau   = s/(1.0+c);
			tM    = t*M(i,j);

#define rot(a, s, t, i, j, k, l) \
			{ g=a(i,j); h=a(k,l); a(i,j)=g-s*(h+g*t); a(k,l)=h+s*(g-h*t); }

			M(i,j)  = 0.0;
			for (k=  0; k<i; ++k)  rot(M, s, tau, k, i, k, j);
			for (k=i+1; k<j; ++k)  rot(M, s, tau, i, k, k, j);
			for (k=j+1; k<4; ++k)  rot(M, s, tau, i, k, j, k);
			for (k=  0; k<4; ++k)  rot(V, s, tau, k, i, k, j);
			M(i,i) -= tM;
			M(j,j) += tM;
		}
	}


	// did it work?
	if (!iter)
	{
		std::cerr << "registration: Jacobi did not converge\n";
		return Mat4f::Identity();
	}


	// eigenvector wrt largest eigenvalue -> quaternion
	Vec4d q;
	{
		int imax=0;
		double s, ss = M(imax,imax);
		if ( (s=M(1,1)) > ss) { ss=s; imax=1; }
		if ( (s=M(2,2)) > ss) { ss=s; imax=2; }
		if ( (s=M(3,3)) > ss) { ss=s; imax=3; }
		q = Vec4d( V(0,imax), V(1,imax), V(2,imax), V(3,imax) );
		q.normalize();
	}


	// rotation part
	float
		ww(q[0]*q[0]), xx(q[1]*q[1]), yy(q[2]*q[2]), zz(q[3]*q[3]),
		wx(q[0]*q[1]), wy(q[0]*q[2]), wz(q[0]*q[3]),
		xy(q[1]*q[2]), xz(q[1]*q[3]), yz(q[2]*q[3]);
	Mat4f T;
	T(0,0) = ww + xx - yy - zz;
	T(1,0) = 2.0*(xy + wz);
	T(2,0) = 2.0*(xz - wy);
	T(3,0) = 0.0;
	T(0,1) = 2.0*(xy - wz);
	T(1,1) = ww - xx + yy - zz;
	T(2,1) = 2.0*(yz + wx);
	T(3,1) = 0.0;
	T(0,2) = 2.0*(xz + wy);
	T(1,2) = 2.0*(yz - wx);
	T(2,2) = ww - xx - yy + zz;
	T(3,2) = 0.0;



	// translation part
	T(0,3) = dcog[0] - T(0,0)*scog[0] - T(0,1)*scog[1] - T(0,2)*scog[2];
	T(1,3) = dcog[1] - T(1,0)*scog[0] - T(1,1)*scog[1] - T(1,2)*scog[2];
	T(2,3) = dcog[2] - T(2,0)*scog[0] - T(2,1)*scog[1] - T(2,2)*scog[2];
	T(3,3) = 1.0;


	return T;
}

//===========================================================================//

void ICP::alignWithWristband(
	std::vector<Vec3f> &points,
	std::vector<Vec3f> &wristband,
	Cylinders &cylinders,
	int iterRigid, int iterSkeletal, int iterIK,
	float thresholdRigid, float thresholdSkeletal)
{
	reset();

	if(points.size() < 3)
		return;

	prediction(cylinders.getSkeleton());

	alignRigid(points, cylinders, iterRigid, thresholdRigid);

	// skeletal icp, separate wristband and hand correspondences
	{
		if(iterSkeletal < 1)
			return;

		float e = thresholdSkeletal + 1;
		float pe = 0;

		e  = 0;
		e += correspond(points, cylinders);
		e += correspondPart(wristband, cylinders, "HandForearm");

		if(points.size() < 3 || wristband.size() < 3)
			return;

		for(int j = 0; j < iterIK; ++j)
			ik.solve(*cylinders.getSkeleton());

		int i = 1;

		while(i++ < iterSkeletal && fabs(e - pe) > thresholdSkeletal)
		{
			pe = e;
			e  = 0;
			e += correspond(points, cylinders);
			e += correspondPart(wristband, cylinders, "HandForearm");

			if(points.size() < 3 || wristband.size() < 3)
				return;

			for(int j = 0; j < iterIK; ++j)
				ik.solve(*cylinders.getSkeleton());
		}
	}

	cylinders.update();
}

//===========================================================================//

float ICP::correspondPart(
	std::vector<Vec3f> &points,
	Cylinders &cylinders,
	const std::string &jointName)
{
	float e = 0;
	cylinders.update();
	std::vector<Vec3f> effectorsPart(points.size());
	std::vector<bool> remove(points.size(), true);

#pragma omp parallel for
	for(size_t i = 0; i < points.size(); ++i)
	{
		Vec3f p, n;

		int id = cylinders.getSkeleton()->getID(jointName);
		float d = cylinders.getSegmentByID(id).distanceTo(points[i], p, n);

		if(d < 50) remove[i] = false;
		else e += d*d;

		effectorsPart[i] = p;
	}

	Skeleton *skeleton = cylinders.getSkeleton();

	for(size_t i = points.size(); i-->0;)
	{
		if(remove[i])
		{
			points.erase(points.begin() + i);
			effectorsPart.erase(effectorsPart.begin() + i);
		}
		else
		{
			Joint *j = skeleton->getJoint(jointName);
			Effector *e = new Effector(j, effectorsPart[i], points[i]);
			skeleton->addEffector(e);
		}
	}

	effectors.insert(effectors.end(), effectorsPart.begin(), effectorsPart.end());

	return e;
}

//===========================================================================//

bool icp_invalid(const Vec3f &s, const Vec3f &t, const Vec3f &n, const Vec3f &zero)
{
    float thresh = 50.0f;

    bool s_invalid =
        ( s == zero
          || isnan(s[0])
          || isnan(s[1])
          || isnan(s[2]) );

    bool t_invalid =
        ( t == zero
          || isnan(t[0])
          || isnan(t[1])
          || isnan(t[2]) );

    bool n_invalid =
        ( n == zero
          || isnan(n[0])
          || isnan(n[1])
          || isnan(n[2]) );

    bool st_invalid = ( (s-t).norm() > thresh );

    return s_invalid || t_invalid || st_invalid || n_invalid;
}

//---------------------------------------------------------------------------//

void ICP::rigidICP(
    Cylinders &model,
    const std::vector<Vec3f> &points,
    int iterations)
{
    if(points.size() < 3)
        return;

    for(int i = 0; i < iterations; ++i)
    {
        Vec3f zero(0,0,0);
        vector<Vec3f> target(points.size(), zero);
        vector<Vec3f> source(points.size(), zero);
        vector<Vec3f> normal(points.size(), zero);

#pragma omp parallel for
        for(size_t j = 0; j < points.size(); ++j)
        {
            Vec3f s, n, t = points[j];
            std::string name;

            float d = model.nearest(t, s, n, name);

            if(!isnan(d) && d < 50.0f)
            {
                target[j] = t;
                source[j] = s;
                normal[j] = n;
            }
        }

        for(size_t j = points.size(); j-->0;)
        {
            if(icp_invalid(source[j], target[j], normal[j], zero))
            {
                target.erase(target.begin() + j);
                source.erase(source.begin() + j);
                normal.erase(normal.begin() + j);
            }
            else
            {
                // project target onto line along the source's normal for orthogonality
                //target[j] = source[j] + (target[j] - source[j]).dot(normal[j]) * normal[j];
            }
        }

        if(target.size() >= 3 && source.size() >= 3)
        {
            model.transform(registration(source, target));
        }
    }
}

//===========================================================================//

