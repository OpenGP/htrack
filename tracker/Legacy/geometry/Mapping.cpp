#include "Mapping.h"

#include <iostream>
#include <iomanip>
#include "../math/PCA.h"

using namespace std;
using namespace Eigen;

//===========================================================================//

Mapping::Mapping(int num)
{
	count = 0;
	numParameters = num;
	numFixedDof = 0;
	numDof = 0;

	enablePCA = false;
	hasPose = false;
	hasScale = false;

	poseJoint = "";
	scaleJoint = "";

	rotationAxis1 = Vec3f::UnitX();
	rotationAxis2 = Vec3f::UnitY();
	rotationAxis3 = Vec3f::UnitZ();

	e1 = 0;
	e2 = 1;
	e3 = 2;

	serial_ptr = NULL;
    pca = new PCA();
}

//===========================================================================//

void Mapping::init()
{
	withPose();
	withScale();
}

//===========================================================================//

void Mapping::map(
	int index,
	const std::string &joint,
	const Vec3f &axis,
	AxisType type)
{
	axisInfos[joint].push_back(AxisInfo(axis, index, type));
	jointInfos[index] = JointInfo(joint, axis, type, index);

	if(++count > numParameters)
		numParameters = count;
}

//===========================================================================//

void Mapping::map(
	int index,
	const std::string &joint,
	const Vec3f &axis,
	AxisType type,
	float min,
	float max)
{
	map(index, joint, axis, type);

	mins[index] = min;
	maxs[index] = max;
}

//===========================================================================//

void Mapping::withPose(
	const std::string &joint,
	int r1, int r2, int r3)
{
	hasPose = true;
	poseJoint = joint;

	rotationAxis1 = Vec3f::Unit(r1);
	rotationAxis2 = Vec3f::Unit(r2);
	rotationAxis3 = Vec3f::Unit(r3);

	e1 = r1;
	e2 = r2;
	e3 = r3;

	map(0, poseJoint, Vec3f::UnitX(), TRANSLATION_AXIS);
	map(1, poseJoint, Vec3f::UnitY(), TRANSLATION_AXIS);
	map(2, poseJoint, Vec3f::UnitZ(), TRANSLATION_AXIS);

	map(3, poseJoint, rotationAxis1, ROTATION_AXIS);
	map(4, poseJoint, rotationAxis2, ROTATION_AXIS);
	map(5, poseJoint, rotationAxis3, ROTATION_AXIS);

	setFixedDof(6);
}

//===========================================================================//

void Mapping::withScale(
	const std::string &joint)
{
	hasScale = true;
	scaleJoint = joint;
}

//===========================================================================//

void Mapping::defaults(
        bool definePose, bool defineForearm, bool defineTwist,
        Vec3f X_AXIS, Vec3f Y_AXIS, Vec3f Z_AXIS)
{
    int idx;

	if(definePose)
	{
		init();
		idx = 6;
	}
	else
	{
		withScale();
		idx = 0;
	}

	if(defineForearm)
	{
		map(idx++, "HandForearm", Z_AXIS, ROTATION_AXIS, -0.25, 0.25);
		map(idx++, "HandForearm", X_AXIS, ROTATION_AXIS, -0.25, 0.25);
		setFixedDof(8);
	}

    if(defineTwist)
    {
//#define WITH_THUMB_TWIST
#ifdef WITH_THUMB_TWIST
        map(idx++, "HandThumb1", Y_AXIS, ROTATION_AXIS, 0, 0.8);
#else
        map(idx++, "", Y_AXIS, ROTATION_AXIS, 0, 0); // keep number of dofs at 29
#endif
        setFixedDof(9);
    }

#if 1
    // conservative joint bounds
    map(idx++, "HandThumb1", Z_AXIS, ROTATION_AXIS, -0.3, 0.3);
	map(idx++, "HandThumb1", X_AXIS, ROTATION_AXIS, -1.5, 0.1);
	map(idx++, "HandThumb2", X_AXIS, ROTATION_AXIS, -1.5, 0.1);
	map(idx++, "HandThumb3", X_AXIS, ROTATION_AXIS, -1.5, 0.1);

	map(idx++, "HandIndex1", Z_AXIS, ROTATION_AXIS, -0.5, 0.4);
	map(idx++, "HandIndex1", X_AXIS, ROTATION_AXIS, -1.5, 0.1);
	map(idx++, "HandIndex2", X_AXIS, ROTATION_AXIS, -1.5, 0.1);
	map(idx++, "HandIndex3", X_AXIS, ROTATION_AXIS, -1.5, 0.1);

	map(idx++, "HandMiddle1", Z_AXIS, ROTATION_AXIS, -0.3, 0.3);
	map(idx++, "HandMiddle1", X_AXIS, ROTATION_AXIS, -1.5, 0.1);
	map(idx++, "HandMiddle2", X_AXIS, ROTATION_AXIS, -1.5, 0.1);
	map(idx++, "HandMiddle3", X_AXIS, ROTATION_AXIS, -1.5, 0.1);

	map(idx++, "HandRing1", Z_AXIS, ROTATION_AXIS, -0.3, 0.3);
	map(idx++, "HandRing1", X_AXIS, ROTATION_AXIS, -1.5, 0.1);
	map(idx++, "HandRing2", X_AXIS, ROTATION_AXIS, -1.5, 0.1);
	map(idx++, "HandRing3", X_AXIS, ROTATION_AXIS, -1.5, 0.1);

	map(idx++, "HandPinky1", Z_AXIS, ROTATION_AXIS, -0.5, 0.3);
	map(idx++, "HandPinky1", X_AXIS, ROTATION_AXIS, -1.5, 0.1);
	map(idx++, "HandPinky2", X_AXIS, ROTATION_AXIS, -1.5, 0.1);
	map(idx++, "HandPinky3", X_AXIS, ROTATION_AXIS, -1.5, 0.1);
#elif 0
    // more liberal joint bounds
    map(idx++, "HandThumb1", Z_AXIS, ROTATION_AXIS, -0.4, 0.4);
	map(idx++, "HandThumb1", X_AXIS, ROTATION_AXIS, -1.5, 0.1);
	map(idx++, "HandThumb2", X_AXIS, ROTATION_AXIS, -1.5, 0.1);
	map(idx++, "HandThumb3", X_AXIS, ROTATION_AXIS, -1.5, 0.1);

	map(idx++, "HandIndex1", Z_AXIS, ROTATION_AXIS, -0.4, 0.4);
	map(idx++, "HandIndex1", X_AXIS, ROTATION_AXIS, -1.5, 0.1);
	map(idx++, "HandIndex2", X_AXIS, ROTATION_AXIS, -1.5, 0.1);
	map(idx++, "HandIndex3", X_AXIS, ROTATION_AXIS, -1.5, 0.1);

	map(idx++, "HandMiddle1", Z_AXIS, ROTATION_AXIS, -0.4, 0.4);
	map(idx++, "HandMiddle1", X_AXIS, ROTATION_AXIS, -1.5, 0.1);
	map(idx++, "HandMiddle2", X_AXIS, ROTATION_AXIS, -1.5, 0.1);
	map(idx++, "HandMiddle3", X_AXIS, ROTATION_AXIS, -1.5, 0.1);

	map(idx++, "HandRing1", Z_AXIS, ROTATION_AXIS, -0.4, 0.4);
	map(idx++, "HandRing1", X_AXIS, ROTATION_AXIS, -1.5, 0.1);
	map(idx++, "HandRing2", X_AXIS, ROTATION_AXIS, -1.5, 0.1);
	map(idx++, "HandRing3", X_AXIS, ROTATION_AXIS, -1.5, 0.1);

	map(idx++, "HandPinky1", Z_AXIS, ROTATION_AXIS, -0.4, 0.4);
	map(idx++, "HandPinky1", X_AXIS, ROTATION_AXIS, -1.5, 0.1);
	map(idx++, "HandPinky2", X_AXIS, ROTATION_AXIS, -1.5, 0.1);
    map(idx++, "HandPinky3", X_AXIS, ROTATION_AXIS, -1.5, 0.1);
#else
    // very liberal joint bounds
    map(idx++, "HandThumb1", Z_AXIS, ROTATION_AXIS, -0.8, 0.8);
	map(idx++, "HandThumb1", X_AXIS, ROTATION_AXIS, -1.6, 0.4);
	map(idx++, "HandThumb2", X_AXIS, ROTATION_AXIS, -1.6, 0.4);
	map(idx++, "HandThumb3", X_AXIS, ROTATION_AXIS, -1.6, 0.4);

	map(idx++, "HandIndex1", Z_AXIS, ROTATION_AXIS, -0.8, 0.8);
	map(idx++, "HandIndex1", X_AXIS, ROTATION_AXIS, -1.6, 0.4);
	map(idx++, "HandIndex2", X_AXIS, ROTATION_AXIS, -1.6, 0.4);
	map(idx++, "HandIndex3", X_AXIS, ROTATION_AXIS, -1.6, 0.4);

	map(idx++, "HandMiddle1", Z_AXIS, ROTATION_AXIS, -0.8, 0.8);
	map(idx++, "HandMiddle1", X_AXIS, ROTATION_AXIS, -1.6, 0.4);
	map(idx++, "HandMiddle2", X_AXIS, ROTATION_AXIS, -1.6, 0.4);
	map(idx++, "HandMiddle3", X_AXIS, ROTATION_AXIS, -1.6, 0.4);

	map(idx++, "HandRing1", Z_AXIS, ROTATION_AXIS, -0.8, 0.8);
	map(idx++, "HandRing1", X_AXIS, ROTATION_AXIS, -1.6, 0.4);
	map(idx++, "HandRing2", X_AXIS, ROTATION_AXIS, -1.6, 0.4);
	map(idx++, "HandRing3", X_AXIS, ROTATION_AXIS, -1.6, 0.4);

	map(idx++, "HandPinky1", Z_AXIS, ROTATION_AXIS, -0.8, 0.8);
	map(idx++, "HandPinky1", X_AXIS, ROTATION_AXIS, -1.6, 0.4);
	map(idx++, "HandPinky2", X_AXIS, ROTATION_AXIS, -1.6, 0.4);
    map(idx++, "HandPinky3", X_AXIS, ROTATION_AXIS, -1.6, 0.4);
#endif
}

//===========================================================================//

void Mapping::setDefaults(bool definePose, bool defineForearm, bool defineTwist)
{
	Vec3f X_AXIS(1,0,0);
	Vec3f Y_AXIS(0,1,0);
	Vec3f Z_AXIS(0,0,1);

	defaults(definePose, defineForearm, defineTwist, X_AXIS, Y_AXIS, Z_AXIS);
}

//===========================================================================//

void Mapping::setDefaultsR(bool definePose, bool defineForearm, bool defineTwist)
{
	Vec3f X_AXIS(1,0,0);
	Vec3f Y_AXIS(0,-1,0);
	Vec3f Z_AXIS(0,0,-1);
    
    defaults(definePose, defineForearm, defineTwist, X_AXIS, Y_AXIS, Z_AXIS);
}

//===========================================================================//

Vec3f Mapping::getRotationAxis(int i)
{
	if(i == 0)
		return rotationAxis1;
	if(i == 1)
		return rotationAxis2;
	if(i == 2)
		return rotationAxis3;
	else
		return Vec3f(0,0,0);
}

//===========================================================================//

Vector3i Mapping::getRotationAxes()
{
	return Vector3i(e1, e2, e3);
}

//===========================================================================//

Vec3f Mapping::toEulerAngles(const Mat3f &m)
{
	return toEuler(m, e1, e2, e3);
}

//===========================================================================//

Vec3f Mapping::toEulerAngles(const Mat4f &m)
{
	return toEuler(m, e1, e2, e3);
}

//===========================================================================//

Mat3f Mapping::fromEulerAngles(const Vec3f &v)
{
	return fromEuler(v, e1, e2, e3);
}

//===========================================================================//

Mat3f Mapping::fromEulerAngles(float a, float b, float c)
{
	return fromEuler(a, b, c, e1, e2, e3);
}

//===========================================================================//

// pca usage:
// - call doPCA with database, desired pc space dimension and fixed dof.
// - fixed dof are the parts of the kinematic model that are unaffected by the
//   pc space mapping because they are not contained in the database. they must
//   be defined as the first elements of the joint angle / pc space vectors.
//   for me, these are the six global translation / rotation parameters.
// - use fromPC and toPC to convert between joint angle and pc space.

void Mapping::doPCA(
	const vector<vector<float> > &data,
	bool setLimits, size_t dof, size_t fDofData)
{
    if(pca)
        delete pca;
    pca = new PCA(data, dof, numFixedDof, fDofData);

	enablePCA = true;
	numDof = dof;

	if(setLimits)
		setParameterLimits(data, dof, fDofData);

    cout << "performed pca on data set" << endl;

    const MatXf &pcs = getPCs();
    cout << "principal components:" << endl << pcs
         << endl << endl << "eigenvalues:" << endl;

    const vector<float> &eig = getEigenvalues();
    float sum = 0;
    
    for(size_t i = 0; i < eig.size(); ++i)
    {
        cout << eig[i] << " ";
        sum += eig[i];
    }

    cout << endl << endl << "variance distribution:" << endl;

    for(size_t i = 0; i < eig.size(); ++i)
    {
        float p = eig[i] / sum;
        cout << setfill(' ') << setw(2) << i+1 << ": "
             << fixed << setprecision(6) << p << " "
             << setfill('.') << setw(p * 100) << "." << endl;
    }

    cout << endl << "integrated distribution:" << endl;

    float percent = 0;
    
    for(size_t i = 0; i < eig.size(); ++i)
    {
        percent += eig[i] / sum;
        cout << setfill(' ') << setw(2) << i+1 << ": "
             << fixed << setprecision(6) << percent << " "
             << setfill('.') << setw(percent * 50) << "." << endl;
    }

    cout << setfill(' ') << endl;
}

//===========================================================================//

void Mapping::adaptPCA(
	const vector<float> &posture)
{
    VecXf sample = pca->adapt(posture);
    numDof = pca->numDof;

	// update parameter limits for corrective dofs
    if(pca->isValid(sample))
	{
		const vector<float> &s = toPC(posture);

        for(size_t i = pca->as.cols(); i < s.size(); ++i)
		{
			if(minsPC.find(i) == minsPC.end())
				minsPC[i] = s[i];
			else if(minsPC[i] > s[i])
				minsPC[i] = s[i];

			if(maxsPC.find(i) == maxsPC.end())
				maxsPC[i] = s[i];
			else if(maxsPC[i] < s[i])
				maxsPC[i] = s[i];
		}
	}
}

//===========================================================================//

void Mapping::resetPCA()
{
	if(!enablePCA)
		return;
    pca->reset();
    numDof = pca->numDof;
}

//===========================================================================//

void Mapping::expandPCA()
{
	if(!enablePCA)
		return;
    pca->expand();
    numDof = pca->numDof;
}

//===========================================================================//

void resetMap(map<int, float> &m, size_t f, size_t n, float val)
{
	for(size_t i = f; i < f + n; ++i)
		m[i] = val;
}

//===========================================================================//

void Mapping::setParameterLimits(
	const vector<vector<float> > &data,
	size_t dof, size_t fDofData)
{
	size_t num = data.size();
	if(!num)
		return;

	size_t dim = data[0].size() - fDofData;
	if(!dof)
		dof = dim;

	resetMap(mins, numFixedDof, dim, numeric_limits<float>::max());
	resetMap(maxs, numFixedDof, dim, numeric_limits<float>::lowest());

	if(enablePCA)
	{
		resetMap(minsPC, numFixedDof, dof, numeric_limits<float>::max());
		resetMap(maxsPC, numFixedDof, dof, numeric_limits<float>::lowest());

		for(int i = 0; i < numFixedDof; ++i)
		{
			if(hasLimits(i))
			{
				minsPC[i] = mins[i];
				maxsPC[i] = maxs[i];
			}
		}
	}

	for(size_t i = 0; i < num; ++i)
	{
		vector<float> dat(numFixedDof + dim, 0);
		for(size_t j = numFixedDof; j < numFixedDof + dim; ++j)
			dat[j] = data[i][j - (numFixedDof - fDofData)];

		for(size_t j = numFixedDof; j < numFixedDof + dim; ++j)
		{
			float val = dat[j];

			if(val < mins[j])
				mins[j] = val;

			if(val > maxs[j])
				maxs[j] = val;
		}

		if(enablePCA)
		{
			const vector<float> &pc = toPC(dat);

			for(size_t j = numFixedDof; j < numFixedDof + dof; ++j)
			{
				float val = pc[j];

				if(val < minsPC[j])
					minsPC[j] = val;

				if(val > maxsPC[j])
					maxsPC[j] = val;
			}
		}
	}

	//-- debug output

	cout << "new parameter limits:" << endl;
	cout << " joint angle space:" << endl;

	for(size_t i = numFixedDof; i < numFixedDof + dim; ++i)
		cout << "  " << i << ": [" << mins[i] << ", " << maxs[i] << "]" << endl;
	
	if(enablePCA)
	{
		cout << " pca space:" << endl;
		for(size_t i = numFixedDof; i < numFixedDof + dof; ++i)
			cout << "  " << i << ": [" << minsPC[i] << ", " << maxsPC[i] << "]" << endl;
	}
	
	//cout << endl;

	//--
}

//===========================================================================//

std::vector<float> Mapping::toPC(const vector<float> &param, size_t newDof)
{
	if(!enablePCA)
		return param;
	else
        return pca->toPC(param, newDof);
}

//===========================================================================//

std::vector<float> Mapping::fromPC(const vector<float> &param, size_t newDof)
{
	if(!enablePCA)
		return param;
	else
        return pca->fromPC(param, newDof);
}

//===========================================================================//

MatXf Mapping::getPCs(size_t dof)
{
    return pca->getPCs(dof);
}

//===========================================================================//
	
std::vector<float> Mapping::getEigenvalues()
{
    return pca->eig;
}

//===========================================================================//
	
std::vector<float> Mapping::getMidpoint()
{
    return pca->mid;
}

PCA &Mapping::getPCA() { return *pca; }

//===========================================================================//

std::vector<AxisInfo> Mapping::getAxisInfos(const std::string &joint)
{
	if(axisInfos.find(joint) != axisInfos.end())
		return axisInfos[joint];
	else
		return std::vector<AxisInfo>();
}

//===========================================================================//

JointInfo Mapping::getJointInfo(int index)
{
	if(jointInfos.find(index) != jointInfos.end())
		return jointInfos[index];
	else
		return JointInfo();
}

//===========================================================================//

std::vector<float> Mapping::constrain(const std::vector<float> &param)
{
	vector<float> result(param.size());

	for(size_t i = 0; i < param.size(); ++i)
	{
		float qmin = getMin(i);
		float qmax = getMax(i);

		float q = param[i];
		float e = 0.001;

		if(q < qmin)
			result[i] = qmin + e;
		else if(q > qmax)
			result[i] = qmax - e;
		else
			result[i] = q;
	}

	return result;
}

//===========================================================================//

float Mapping::constrain(int index, float current, float delta)
{
	if(mins.find(index) == mins.end() || maxs.find(index) == maxs.end())
	{
		return delta;
	}

	if(current + delta > maxs[index])
	{
		return maxs[index] - current;
	}

	if(current + delta < mins[index])
	{
		return current - mins[index];
	}

	return delta;
}

//===========================================================================//

float Mapping::getMin(int index)
{
	if(mins.find(index) != mins.end())
		return mins[index];
	else
		//return std::numeric_limits<float>::min();
		return -std::numeric_limits<float>::max();
}

//===========================================================================//

float Mapping::getMax(int index)
{
	if(maxs.find(index) != maxs.end())
		return maxs[index];
	else
		return std::numeric_limits<float>::max();
}

//===========================================================================//

std::vector<float> Mapping::constrainPC(const std::vector<float> &param)
{
	vector<float> result(param.size());

	for(size_t i = 0; i < param.size(); ++i)
	{
		float qmin = getMinPC(i);
		float qmax = getMaxPC(i);

		float q = param[i];
		float e = 0.01;

		if(q < qmin)
			result[i] = qmin + e;
		else if(q > qmax)
			result[i] = qmax - e;
		else
			result[i] = q;
	}

	return result;
}

//===========================================================================//

float Mapping::constrainPC(int index, float current, float delta)
{
	if(minsPC.find(index) == minsPC.end() || maxsPC.find(index) == maxsPC.end())
	{
		return delta;
	}

	if(current + delta > maxsPC[index])
	{
		return maxsPC[index] - current;
	}

	if(current + delta < minsPC[index])
	{
		return current - minsPC[index];
	}

	return delta;
}

//===========================================================================//

float Mapping::getMinPC(int index)
{
	if(minsPC.find(index) != minsPC.end())
		return minsPC[index];
	else
		return std::numeric_limits<float>::min();
}

//===========================================================================//

float Mapping::getMaxPC(int index)
{
	if(maxsPC.find(index) != maxsPC.end())
		return maxsPC[index];
	else
		return std::numeric_limits<float>::max();
}

//===========================================================================//

int Mapping::getNumberOfParameters()
{
	return numParameters;
}

//===========================================================================//

int Mapping::getDof()
{
	return numDof;
}

//===========================================================================//

int Mapping::getFixedDof()
{
	return numFixedDof;
}

//===========================================================================//

void Mapping::setFixedDof(int fDof)
{
	numFixedDof = fDof;
}

//===========================================================================//

void Mapping::setPCAEnabled(bool b)
{
	enablePCA = b;
}

//===========================================================================//

bool Mapping::getPCAEnabled()
{
    return enablePCA && pca->pcs.size() > 0;
}

//===========================================================================//

bool Mapping::hasPoseJoint()
{
	return hasPose;
}

//===========================================================================//

bool Mapping::hasScaleJoint()
{
	return hasScale;
}

//===========================================================================//

std::string Mapping::getPoseJoint()
{
	return poseJoint;
}

//===========================================================================//

std::string Mapping::getScaleJoint()
{
	return scaleJoint;
}

//===========================================================================//

float *Mapping::serialize()
{
	int r,c;
	return serialize(r,c);
}

//===========================================================================//

float *Mapping::serialize(int &rows, int &cols)
{
	size_t r = 4;
	size_t c = jointInfos.size();
	size_t n = r * c;

	rows = r;
	cols = c;

	if(serial_ptr)
		delete serial_ptr;

	serial_ptr = new float[n];

	for(size_t i = 0; i < c; ++i)
	{
		JointInfo info = jointInfos[i];

		size_t k = 0;

		std::cout << i << " ---> " << info.type << std::endl;
		serial_ptr[i*r + k++] = (float)info.type;
		serial_ptr[i*r + k++] = info.axis[0];
		serial_ptr[i*r + k++] = info.axis[1];
		serial_ptr[i*r + k++] = info.axis[2];
	}

	return serial_ptr;
}

