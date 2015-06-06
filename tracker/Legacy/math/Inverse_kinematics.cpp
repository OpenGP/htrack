#include "Inverse_kinematics.h"

#include "../util/Util.h"
#include "MathUtils.h"
#include <iostream>
#include "../util/openmp_helpers.h"

#define SQ(X) ((X)*(X))

using namespace std;
using namespace Eigen;

//===========================================================================//

Inverse_kinematics::Inverse_kinematics(float lambda)
	: lambda(lambda), error(0), scaleVal(1.0f), dof(0), doLS(true),
	  altInv(false), clamp(true), pplane(false), doPredict(true),
	  keepDof(false), singleDof(false), doScale(false)
{
}

//===========================================================================//

void Inverse_kinematics::solve(Skeleton &skeleton)
{
	// predict based on previous estimate
	if(doPredict)
		predict(skeleton);

	// build jacobian matrix
	setup(skeleton);

	// compute parameter update vector
	vector<float> dt = compute(skeleton);

	Mapping &m = skeleton.getMapping();

	const vector<float> &pc = m.toPC(skeleton.getCurrentParameters(), keepDof ? 0 : dof);
	const vector<float> &ps = skeleton.getUpdatedParameters(pc, dt, keepDof);

	params = pc;

	// update skeleton forward kinematics
	if(clamp)
		skeleton.set(m.constrain(m.fromPC(ps, keepDof ? 0 : dof)));
	else
		skeleton.set(m.fromPC(ps, keepDof ? 0 : dof));

	// update scaling
	if(doScale)
		skeleton.setScale(skeleton.getScale() + scaleVal);

	// optionally do step-length control
	if(doLS)
		lineSearch(skeleton, dt);
}

//===========================================================================//

void Inverse_kinematics::predict(Skeleton &skeleton)
{
	if(dthetaPrev.empty())
		return;

	vector<float> dt = factorVec(dthetaPrev, 1.0);

	if(skeleton.hasPoseJoint())
	{
		const vector<int> &is = skeleton.getPoseJointIndices();

		// translation dofs = 0?

		dt[is[3]] = 0;
		dt[is[4]] = 0;
		dt[is[5]] = 0;
	}

	Mapping &m = skeleton.getMapping();

	const vector<float> &pc = skeleton.getCurrentParameters();
	const vector<float> &ps = skeleton.getUpdatedParameters(pc, dt);

	if(clamp)
		skeleton.set(m.constrain(ps));
	else
		skeleton.set(ps);

	doPredict = false;
}

//===========================================================================//

void Inverse_kinematics::setup(Skeleton &skeleton)
{
	Mapping mapping = skeleton.getMapping();
	int num = mapping.getNumberOfParameters();
	int n;

	if(pplane)
		n = skeleton.getNumberOfEffectors();
	else
		n = 3 * skeleton.getNumberOfEffectors();

	// setup damped least squares system
	errors = MatXf::Zero(n, 1);
	jacobian = MatXf::Zero(n, num);

	// scale gradients
	MatXf scaleJ = MatXf::Zero(n, 1);

	// sum of squares error
	error = 0;

	// iterate over constraints
	vector<Effector *> &es = skeleton.getEffectors_();

    #pragma omp parallel for
	for(size_t i = 0; i < es.size(); ++i)
	{
		Vec3f position = es[i]->getPosition();
		Vec3f target = es[i]->getTarget();
		Vec3f normal = es[i]->getTargetNormal(); // for point-plane distance

		// compute errors
		if(pplane)
		{
			errors(i, 0) = (target - position).dot(normal);
		}
		else
		{
			errors(i * 3 + 0, 0) = target[0] - position[0];
			errors(i * 3 + 1, 0) = target[1] - position[1];
			errors(i * 3 + 2, 0) = target[2] - position[2];
		}

		float diff = (target - position).dot(target - position);
		error += diff;

		// backtrack kinematic chain
		Joint joint = *es[i]->getJoint();

		while(joint.getParent() != NULL)
		{
			// scale joint
			if(doScale && !joint.getName().compare(skeleton.getScaleJoint()->getName()))
			{
				Mat4f Tinv = joint.getParent()->getGlobalTransformation().inverse();
				Vec3f grad = Transform3f(Tinv) * position;

				scaleJ(i * 3 + 0, 0) = grad[0];
				scaleJ(i * 3 + 1, 0) = grad[1];
				scaleJ(i * 3 + 2, 0) = grad[2];
			}

			// get all local axes defined for this joint
			const vector<AxisInfo> &infos = mapping.getAxisInfos(joint.getName());

			if(infos.size())
			for(vector<AxisInfo>::const_iterator it = infos.begin(); it != infos.end(); ++it)
			{
				switch(it->type)
				{
					case TRANSLATION_AXIS:
					{
						// calculate global rotation axis
						Vec3f loc = it->axis;
						Mat3f rot = joint.getParent()->getGlobalRotation();
						Vec3f axis = rot * loc;

						// calculate gradient
						Vec3f grad = axis;

						// fill jacobian
						if(pplane)
						{
							jacobian(i, it->index) = grad.dot(normal);
						}
						else
						{
							jacobian(i * 3 + 0, it->index) += grad[0];
							jacobian(i * 3 + 1, it->index) += grad[1];
							jacobian(i * 3 + 2, it->index) += grad[2];
						}

						break;
					}
					case ROTATION_AXIS:
					default:
					{
						// calculate global rotation axis
						Vec3f loc = it->axis;
						Mat3f rot = joint.getGlobalRotation();
						Vec3f axis = rot * loc;

						// calculate relative position
						Vec3f rel = position - joint.getGlobalTranslation();

						// calculate gradient
						Vec3f grad = axis.cross(rel);

						// fill jacobian
						if(pplane)
						{
							jacobian(i, it->index) = grad.dot(normal);
						}
						else
						{
							jacobian(i * 3 + 0, it->index) = grad[0];
							jacobian(i * 3 + 1, it->index) = grad[1];
							jacobian(i * 3 + 2, it->index) = grad[2];
						}

						break;
					}
				}
			}

			joint = *joint.getParent();
		}
	}

	// optimize in pc space
	if(mapping.getPCAEnabled())
		jacobian = jacobian * mapping.getPCs(dof);

	// select single dof
	if(singleDof)
	{
		int c = jacobian.cols();
		MatXf select = MatXf::Zero(c, c);
		select(dof-1, dof-1) = 1.0;
		jacobian = jacobian * select;
	}

	// add scale gradients to jacobian
	if(doScale)
	{
		MatXf tmpJ = jacobian;
		jacobian = MatXf(
			tmpJ.rows(), tmpJ.cols() + 1);
		jacobian.block(
			0, 0,
			tmpJ.rows(), tmpJ.cols() ) = tmpJ;
		jacobian.block(
			0, tmpJ.cols(),
			scaleJ.rows(), scaleJ.cols() ) = scaleJ;
	}
}

//===========================================================================//

MatXf Inverse_kinematics::dampingMatrix(Skeleton &skeleton)
{
	Mapping mapping = skeleton.getMapping();

	int num;
	
	if(mapping.getPCAEnabled())
		num = mapping.getFixedDof() + (dof ? dof : mapping.getDof());
	else
		num = mapping.getNumberOfParameters();

	MatXf invW = MatXf::Identity(num, num);
	vector<float> param;

	if(mapping.getPCAEnabled())
		param = mapping.toPC(skeleton.getCurrentParameters());
	else
		param = skeleton.getCurrentParameters();

	for(int i = 0; i < num; ++i)
	{
		if(!mapping.hasLimits(i))
			continue;

		float qmax, qmin, q, e = 1e-5;

		if(mapping.getPCAEnabled())
		{
			if(!mapping.hasLimitsPC(i))
				continue;

			qmax = mapping.getMaxPC(i);
			qmin = mapping.getMinPC(i);
		}
		else
		{
			qmax = mapping.getMax(i);
			qmin = mapping.getMin(i);
		}

		q = param[i];

		float h = fabs( SQ(qmax - qmin) * (2 * q - qmax - qmin)
				/ ( 4 * SQ(qmax - q) * SQ(q - qmin) ) );

		if(hprev.size() != (size_t)num)
		{
			hprev = vector<float>(num, e);
		}

		float delta = h - hprev[i];
		hprev[i] = h;

		invW(i,i) = delta >= 0 ? (1.0 + h) : 1.0;
	}

	// additional scale dof
	if(doScale)
	{
		MatXf tmpW = invW;
		invW = MatXf::Zero(tmpW.rows() + 1, tmpW.cols() + 1);
		invW.block(0,0,tmpW.rows(),tmpW.cols()) = tmpW;
		invW(tmpW.rows(), tmpW.cols()) = 1.0f;
	}

	// apply damping (except to translational dof)
	for(int i = 0; i < jacobian.cols(); ++i)
	{
		if(mapping.getJointInfo(i).type == TRANSLATION_AXIS)
			invW(i,i) *= 1.0f;
		else
			invW(i,i) *= lambda * lambda;
	}

	return invW;
}

//===========================================================================//

vector<float> Inverse_kinematics::compute(Skeleton &skeleton)
{
	Mapping mapping = skeleton.getMapping();

	int num;
	
	if(mapping.getPCAEnabled())
		num = mapping.getFixedDof() + (dof ? dof : mapping.getDof());
	else
		num = mapping.getNumberOfParameters();

	// solve ik, return parameter update
	MatXf dtheta(num, 1);

	// joint limit avoidance
	if(clamp)
	{
		// solve system with regularized damped pseudo-inverse:
		//  dt = (J^T * J + lambda*W)^-1 * J^T * e
		MatXf invW      = dampingMatrix(skeleton);
		MatXf jacobianT = transp(jacobian);
		MatXf damped    = (jacobianT * jacobian) + invW;
		MatXf inverse   = pinv(damped);
		dtheta = (inverse * jacobianT) * errors;
	}
	// do alternative matrix inversion when dof << constraints
	else if(altInv && jacobian.rows() < jacobian.cols())
	{
		// solve system with damped pseudo-inverse:
		//  dt = J^T * (J * J^T + lambda*I)^-1 * e
		int r = jacobian.rows();
		MatXf id = MatXf::Identity(r, r) * (lambda * lambda);
		MatXf jacobianT = transp(jacobian);
		MatXf damped = (jacobian * jacobianT) + id;
		MatXf inverse = pinv(damped);
		dtheta = (jacobianT * inverse) * errors;
	}
	else // standard solution
	{
		// solve system with damped pseudo-inverse:
		//  dt = (J^T * J + lambda*I)^-1 * J^T * e
		int c = jacobian.cols();
		MatXf id = MatXf::Identity(c, c) * (lambda * lambda);

		// do not apply damping to translational dof
		for(int i = 0; i < jacobian.cols(); ++i)
			if(mapping.getJointInfo(i).type == TRANSLATION_AXIS)
				id(i,i) = 1;

		// compute update
		MatXf jacobianT = transp(jacobian);
		MatXf damped    = (jacobianT * jacobian) + id;
		MatXf inverse   = pinv(damped);

		dtheta = (inverse * jacobianT) * errors;
	}

	// return parameter update
	vector<float> dt(num);
	for(int i = 0; i < num; ++i)
	{
		dt[i] = dtheta(i, 0);
	}

	if(doScale)
		scaleVal = dtheta(num, 0);

	return dt;
}

//===========================================================================//

void Inverse_kinematics::lineSearch(Skeleton &skeleton, vector<float> &dt)
{
	// check whether the joint parameter update decreases the error
	float threshold = 0.001;
	float initialError = error;
	float previousError = error + threshold + 1;

	// repeat until the error decreases or converges
	while(fabs(error - previousError) > threshold)
	{
		// calculate new update error
		previousError = error;
		error = skeleton.getEffectorDistance();

		// if the error did not decrease, try half of the update step
		if(error / initialError > 0.99f)
		{
			Mapping &m = skeleton.getMapping();
			vector<float> pc = m.toPC(skeleton.getCurrentParameters(), keepDof ? 0 : dof);

			if(previousError == initialError || previousError > error)
			{
				// halve the update step
				for(size_t i = 0; i < dt.size(); ++i)
				{
					pc[i] = params[i]; // pc[i] -= dt[i];
					dt[i] /= 2.0;
				}
			}

			// calculate the forward kinematics
			const vector<float> &ps = skeleton.getUpdatedParameters(pc, dt, keepDof);
			if(clamp)
				skeleton.set(m.constrain(m.fromPC(ps, keepDof ? 0 : dof)));
			else
				skeleton.set(m.fromPC(ps, keepDof ? 0 : dof));

			// stop if the regularization increased the error
			if(previousError != initialError && previousError < error)
				return;
		}
		else
		{
			return;
		}
	}
}

//===========================================================================//

vector<float> Inverse_kinematics::factorVec(const vector<float> &v, float factor)
{
	vector<float> fv(v.size());

	for(size_t i = 0; i < v.size(); ++i)
		fv[i] = v[i] * factor;

	return fv;
}

//===========================================================================//

MatXf Inverse_kinematics::constructJacobian(
	Skeleton &skeleton,
	bool parameterLimits_,
	bool targetJacobian_,
	bool pcaEnabled_,
	bool singleDof_,
	bool pplane_)
{
	Mapping mapping = skeleton.getMapping();
	int num = mapping.getNumberOfParameters();
	int n;

	if(pplane_)
		n = skeleton.getNumberOfEffectors();
	else
		n = 3 * skeleton.getNumberOfEffectors();

	// setup damped least squares system
	errors = MatXf::Zero(n, 1);
	jacobian = MatXf::Zero(n, num);

	error = 0;

	// iterate over constraints
	vector<Effector *> &es = skeleton.getEffectors_();

	openmp::setNumThreads(openmp::NUM_THREADS);
    #pragma omp parallel for
	for(size_t i = 0; i < es.size(); ++i)
	{
		Vec3f position = es[i]->getPosition();
		Vec3f target = es[i]->getTarget();
		Vec3f normal = es[i]->getTargetNormal(); // for point-plane distance

		// compute errors
		if(pplane_)
		{
			errors(i, 0) = (target - position).dot(normal);
		}
		else
		{
			errors(i * 3 + 0, 0) = target[0] - position[0];
			errors(i * 3 + 1, 0) = target[1] - position[1];
			errors(i * 3 + 2, 0) = target[2] - position[2];
		}

		float diff = (target - position).dot(target - position);
		error += diff;

		// backtrack kinematic chain
		Joint joint = *es[i]->getJoint();

		while(joint.getParent() != NULL)
		{
			// get all local axes defined for this joint
			const vector<AxisInfo> &infos = mapping.getAxisInfos(joint.getName());

			if(infos.size())
			for(vector<AxisInfo>::const_iterator it = infos.begin(); it != infos.end(); ++it)
			{
				switch(it->type)
				{
					case TRANSLATION_AXIS:
					{
						// calculate global rotation axis
						Vec3f loc = it->axis;
						Mat3f rot = joint.getParent()->getGlobalRotation();
						Vec3f axis = rot * loc;

						// calculate gradient
						Vec3f grad = axis;

						// fill jacobian
						if(pplane_)
						{
							jacobian(i, it->index) = grad.dot(normal);
						}
						else
						{
							jacobian(i * 3 + 0, it->index) = grad[0];
							jacobian(i * 3 + 1, it->index) = grad[1];
							jacobian(i * 3 + 2, it->index) = grad[2];
						}

						break;
					}
					case ROTATION_AXIS:
					default:
					{
						// calculate global rotation axis
						Vec3f loc = it->axis;
						Mat3f rot = joint.getGlobalRotation();
						Vec3f axis = rot * loc;

						// calculate relative position
						Vec3f rel = position - joint.getGlobalTranslation();

						if(targetJacobian_)
						{
							rel = target - joint.getGlobalTranslation();
						}

						// calculate gradient
						Vec3f grad = axis.cross(rel);

						// fill jacobian
						if(pplane_)
						{
							jacobian(i, it->index) = grad.dot(normal);
						}
						else
						{
							jacobian(i * 3 + 0, it->index) = grad[0];
							jacobian(i * 3 + 1, it->index) = grad[1];
							jacobian(i * 3 + 2, it->index) = grad[2];
						}

						break;
					}
				}
			}

			joint = *joint.getParent();
		}
	}

	// stack jacobian and damping matrix
	if(parameterLimits_)
	{
		MatXf invW = MatXf::Zero(num, num);
		vector<float> param;

		param = skeleton.getCurrentParameters();

		int fDof = mapping.getFixedDof();

		for(int i = 0; i < fDof; ++i)
		{
			invW(i,i) = 1.0;
		}

		for(int i = fDof; i < num; ++i)
		{
			float qmax, qmin, q, e = 1e-5;

			qmax = mapping.getMax(i);
			qmin = mapping.getMin(i);

			q = param[i];

			float h = fabs( SQ(qmax - qmin) * (2 * q - qmax - qmin)
						    / ( 4 * SQ(qmax - q) * SQ(q - qmin) ) );

			if(hprev.size() != (size_t)num)
			{
				hprev = vector<float>(num, e);
			}

			float delta = h - hprev[i];
			//hprev[i] = h; // let actual ik update this

			invW(i,i) = delta >= 0 ? (1.0 + h) : 1.0;
		}

		// apply damping (except to translational dof)
		for(int i = 0; i < jacobian.cols(); ++i)
		{
			if(mapping.getJointInfo(i).type == TRANSLATION_AXIS)
				invW(i,i) *= 1.0f;
			else
				invW(i,i) *= lambda * lambda;
		}

		// take square root of matrix elements
		for(int i = 0; i < invW.cols(); ++i)
			invW(i,i) = sqrt(invW(i,i));

		// stack jacobian and damping matrix
		MatXf stacked(
			jacobian.rows() + invW.rows(), jacobian.cols());

		stacked.block(0,0,jacobian.rows(),jacobian.cols()) << jacobian;
		stacked.block(jacobian.rows(),0,invW.rows(),invW.cols()) << invW;

		return stacked; // must return here due to dimensions
	}

	// optimize in pc space
	if(pcaEnabled_)
		jacobian = jacobian * mapping.getPCs(dof);

	// select single dof
	if(singleDof_)
	{
		int c = jacobian.cols();
		MatXf select = MatXf::Zero(c, c);
		select(dof-1, dof-1) = 1.0;
		jacobian = jacobian * select;
	}

	return jacobian;
}

//===========================================================================//

