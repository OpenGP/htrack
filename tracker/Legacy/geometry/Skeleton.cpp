#include "Skeleton.h"

#include <Eigen/Geometry>

#include <iostream>
#include "../util/Util.h"

using namespace std;
using namespace Eigen;

//===========================================================================//

Skeleton::Skeleton(Joint *root, const Mapping &map)
{
	numJoints = 0;
	numEffectors = 0;
	forward = Vec3f(0,1,0);

	joints = std::map<std::string, Joint *>();
	effectors = std::map<std::string, std::vector<Effector *> >();

	joints_ = std::vector<Joint *>();
	effectors_ = std::vector<Effector *>();

	poseJoint = "";
	scaleJoint = "";

	tx = ty = tz = -1;
	rx = ry = rz = -1;

	scaleVal = 1.0;
	
	setRoot(root);
	setMapping(map);
}

//===========================================================================//

Skeleton::Skeleton()
{
	numJoints = 0;
	numEffectors = 0;
	forward = Vec3f(0,1,0);

	current = std::vector<float>(mapping.getNumberOfParameters(), 0);

	joints = std::map<std::string, Joint *>();
	effectors = std::map<std::string, std::vector<Effector *> >();

	joints_ = std::vector<Joint *>();
	effectors_ = std::vector<Effector *>();

	poseJoint = "";
	scaleJoint = "";

	tx = ty = tz = -1;
	rx = ry = rz = -1;

	scaleVal = 1.0;
}

//===========================================================================//

Skeleton::~Skeleton()
{
	clearEffectors();
	clearJoints();
}

//===========================================================================//

void Skeleton::transformJoints(const std::vector<float> &parameters)
{
	// move joints specified in parameters
	for(size_t i = 0; i < parameters.size(); ++i)
	{
		JointInfo info = mapping.getJointInfo(i);
		Joint *joint = getJoint(info.joint);
        if(joint == NULL) continue;

		float x = parameters[i];

		switch(info.type)
		{
			case TRANSLATION_AXIS:
			{
				joint->translate(info.axis * x);
				break;
			}
			case ROTATION_AXIS:
			default:
			{
				joint->rotate(info.axis, x);
				break;
			}
		}

		current[i] += x;
	}
}

//===========================================================================//

void Skeleton::move(const std::vector<float> &parameters)
{
	size_t n = min(parameters.size(), (size_t)mapping.getNumberOfParameters());

	// separate rotation axes and pose parameters
	vector<float> rotateX(n, 0); // flexion
	vector<float> rotateZ(n, 0); // abduction
	vector<float> rotateY(n, 0); // twist
	vector<float> globals(n, 0); // pose

	for(size_t i = 0; i < n; ++i)
	{
		const JointInfo &info = mapping.getJointInfo(i);

		if(    info.joint.compare(poseJoint)   // not pose joint
			&& info.type == ROTATION_AXIS )    // not translation
        {
            Vec3f axis = info.axis.array().abs();

            if(axis == Vec3f(1,0,0))
                rotateX[i] = parameters[i];    // flexion
            else if(axis == Vec3f(0,1,0))
                rotateY[i] = parameters[i];    // abduction
            else if(axis == Vec3f(0,0,1))
                rotateZ[i] = parameters[i];    // twist
            else
                rotateY[i] = parameters[i];    // other
        }
		else
		{
			globals[i] = parameters[i];        // pose
		}
	}

	// transform joints separately
	transformJoints(globals); // pose
	transformJoints(rotateX); // flexion
	transformJoints(rotateZ); // abduction
	transformJoints(rotateY); // twist

	// move effectors of the skeleton
	moveEffectors();
}

//===========================================================================//

void Skeleton::set(const std::vector<float> &parameters)
{
	std::vector<float> p = parameters;
    
    ///--- complete parameter vector to full-dimension
	if(parameters.size() < current.size())
		for(size_t i = parameters.size(); i < current.size(); ++i)
			p.push_back(current[i]);

	reset();
	move(p);
}

//===========================================================================//

void Skeleton::set(VectorN parameters)
{
    std::vector<float> p;
    for (int i = 0; i < parameters.size(); ++i) {
        p.push_back(parameters[i]);
    }
    ///--- complete parameter vector to full-dimension
    if(parameters.size() < current.size())
        for(size_t i = parameters.size(); i < current.size(); ++i)
            p.push_back(current[i]);

    reset();
    move(p);
}

//===========================================================================//

void Skeleton::moveEffectors()
{
	std::map<std::string, Joint *>::iterator it;
	for(it = joints.begin(); it != joints.end(); ++it)
	{
		if(hasEffectors(it->first))
		{
			std::vector<Effector *> &es = effectors[it->first];

			for(size_t j = 0; j < es.size(); ++j)
			{
				const Mat4f &transf = it->second->getGlobalTransformation();
				const Vec3f &offset = es[j]->getOffset();

				es[j]->setPosition(Transform3f(transf) * offset);
			}
		}
	}
}

//===========================================================================//

void Skeleton::rotate(const Vec3f &axis, float angle)
{
	if(poseJoint.empty())
		return;

	joints[poseJoint]->rotate(axis, angle);

	const Mat3f &m = joints[poseJoint]->getRotation();
	const Vec3f &e = mapping.toEulerAngles(m);

	current[rx] = e[0];
	current[ry] = e[1];
	current[rz] = e[2];
}

void Skeleton::translate(const Vec3f &translation)
{
	if(poseJoint.empty())
		return;

	joints[poseJoint]->translate(translation);

	current[tx] += translation[0];
	current[ty] += translation[1];
	current[tz] += translation[2];
}

void Skeleton::transform(const Mat4f &matrix)
{
	if(poseJoint.empty())
		return;

	joints[poseJoint]->transform(matrix);

	const Mat4f &m = joints[poseJoint]->getLocalTransformation();

	current[tx] = m(0,3);
	current[ty] = m(1,3);
	current[tz] = m(2,3);

	const Vec3f &e = mapping.toEulerAngles(m);

	current[rx] = e[0];
	current[ry] = e[1];
	current[rz] = e[2];
}

void Skeleton::scale(float value)
{
	if(scaleJoint.empty())
		return;

	Mat4f m = Mat4f::Identity() * value;
	m(3,3) = 1.0;

	Joint *s = joints[scaleJoint];
	s->transform(m);

	setInitialTransformation(scaleJoint, s->getLocalTransformation());

	scaleVal = value;
}

void Skeleton::setRotation(const Mat3f &matrix)
{
	if(poseJoint.empty())
		return;

	joints[poseJoint]->setRotation(matrix);

	const Mat3f &m = joints[poseJoint]->getRotation();
	const Vec3f &e = mapping.toEulerAngles(m);

	current[rx] = e[0];
	current[ry] = e[1];
	current[rz] = e[2];
}

void Skeleton::setRotation(const Vec3f &axis, float angle)
{
	if(poseJoint.empty())
		return;

	joints[poseJoint]->setRotation(axis, angle);

	const Mat3f &m = joints[poseJoint]->getRotation();
	const Vec3f &e = mapping.toEulerAngles(m);

	current[rx] = e[0];
	current[ry] = e[1];
	current[rz] = e[2];
}

void Skeleton::setTranslation(const Vec3f &translation)
{
	if(poseJoint.empty())
		return;

	joints[poseJoint]->setTranslation(translation);

	current[tx] = translation[0];
	current[ty] = translation[1];
	current[tz] = translation[2];
}

void Skeleton::setTransformation(const Mat4f &matrix)
{
	if(poseJoint.empty())
		return;

	joints[poseJoint]->setTransformation(matrix);

	current[tx] = matrix(0,3);
	current[ty] = matrix(1,3);
	current[tz] = matrix(2,3);

	const Vec3f &e = mapping.toEulerAngles(matrix);

	current[rx] = e[0];
	current[ry] = e[1];
	current[rz] = e[2];
}

void Skeleton::setScale(float value)
{
	if(scaleJoint.empty())
		return;

	Mat4f m = Mat4f::Identity() * value / scaleVal;
	m(3,3) = 1.0;

	Joint *s = joints[scaleJoint];
	s->transform(m);

	setInitialTransformation(scaleJoint, s->getLocalTransformation());

	scaleVal = value;
}

void Skeleton::setScaleValue(float value)
{
	scaleVal = value;
}

//===========================================================================//

void Skeleton::setRoot(Joint *root)
{
	this->root = root;
	addJoint(root);
}

//===========================================================================//

void Skeleton::setPoseJoint(
	const std::string &name,
	int tx, int ty, int tz,
	int rx, int ry, int rz)
{
	this->poseJoint = name;

	this->tx = tx;
	this->ty = ty;
	this->tz = tz;

	this->rx = rx;
	this->ry = ry;
	this->rz = rz;
}

//===========================================================================//

void Skeleton::setScaleJoint(
	const std::string &name)
{
	this->scaleJoint = name;
}

//===========================================================================//

void Skeleton::setForwardAxis(const Vec3f &xyz)
{
	forward = xyz;
}

//===========================================================================//

void Skeleton::setInitialTransformations()
{
	std::map<std::string, Joint *>::iterator j;

	for(j = joints.begin(); j != joints.end(); ++j)
		initialTransformations[j->first] = j->second->getLocalTransformation();

	if(hasPoseJoint())
		initialTransformations[poseJoint] = Mat4f::Identity();
}

//===========================================================================//

void Skeleton::setInitialTransformations(
	const aligned_map<std::string, Mat4f>::type &ts)
{
	initialTransformations = ts;
}

//===========================================================================//

void Skeleton::setInitialTransformation(const std::string &jointName, const Mat4f &m)
{
	if(initialTransformations.empty())
		setInitialTransformations();

	initialTransformations[jointName] = m;
}

//===========================================================================//

void Skeleton::reset(bool reinit)
{
	if(initialTransformations.empty() || reinit)
		setInitialTransformations();

	map<string, Joint *>::iterator j;
	for(j = joints.begin(); j != joints.end(); ++j)
    {
        if(j->second != NULL)
            j->second->setTransformation(initialTransformations[j->first]);
    }
	
	current = vector<float>(current.size(), 0);
}

//===========================================================================//

void Skeleton::resetPosture(bool reinit)
{
    if(initialTransformations.empty() || reinit)
        setInitialTransformations();

    map<string, Joint *>::iterator j;
    for(j = joints.begin(); j != joints.end(); ++j)
    {
        if(j->second != NULL)
        {
            if(!isStructural(j->second) || !j->second->getName().compare(scaleJoint))
                j->second->setTransformation(initialTransformations[j->first]);
        }
    }

    for(size_t i = mapping.getFixedDof(); i < mapping.getNumberOfParameters(); ++i)
        current[i] = 0.0f;
}

//===========================================================================//

void Skeleton::setInitialTranslations()
{
	if(initialTransformations.empty())
		setInitialTransformations();

	for(size_t i = 0; i < joints_.size(); ++i)
	{
		const string &n = joints_[i]->getName();
		const Vec3f  &t = joints_[i]->getTranslation();

		if(initialTransformations.find(n) != initialTransformations.end())
		{
			initialTransformations[n](0,3) = t[0];
			initialTransformations[n](1,3) = t[1];
			initialTransformations[n](2,3) = t[2];
		}
	}

	if(hasPoseJoint())
		initialTransformations[poseJoint] = Mat4f::Identity();
}

//===========================================================================//

aligned_map<std::string, Mat4f>::type Skeleton::getInitialTransformations()
{
	return initialTransformations;
}

//===========================================================================//

Mat4f Skeleton::getInitialTransformation(const string &jointName)
{
	return initialTransformations[jointName];
}

//===========================================================================//

void Skeleton::addJoint(Joint *joint)
{
	joints[ joint->getName() ] = joint;
	joints_.push_back(joint);
	numJoints++;

	std::vector<Joint *> &children = joint->getChildren();
	std::vector<Joint *>::iterator c;

	for(c = children.begin(); c != children.end(); ++c)
	{
		addJoint(*c);
	}
}

//===========================================================================//

void Skeleton::addEffector(Effector *effector)
{
	std::string name = effector->getJoint()->getName();

	if(effectors.find(name) == effectors.end())
	{
		std::vector<Effector *> es;
		effectors[name] = es;
	}

	effectors[name].push_back(effector);
	effectors_.push_back(effector);
	numEffectors++;
}

//===========================================================================//

void Skeleton::addEffector(Effector *effector, size_t index)
{
	Effector *e = NULL;

	if(index > effectors_.size() - 1)
	{
		effectors_.resize(index + 1);
		numEffectors++;
	}
	else
	{
		e = effectors_[index];
	}

	effectors_[index] = effector;

	if(e != NULL)
	{
		std::string name = e->getJoint()->getName();
		size_t del = 0;

		for(size_t i = 0; i < effectors[name].size(); ++i)
		{
			if(effectors[name][i] == e)
			{
				del = i;
				break;
			}
		}

		effectors[name].erase(effectors[name].begin() + del);
		delete e;
	}

	std::string name = effector->getJoint()->getName();

	if(effectors.find(name) == effectors.end())
	{
		std::vector<Effector *> es;
		effectors[name] = es;
	}

	effectors[name].push_back(effector);
}

//===========================================================================//

void Skeleton::clearJoints()
{
	if(joints_.size() == 0)
		return;

	for(size_t i = 0; i < joints_.size(); ++i)
		delete joints_[i];

	joints_.clear();
	joints.clear();
	numJoints = 0;
}

//===========================================================================//

void Skeleton::removeEffector(size_t index)
{
	if(index >= effectors_.size())
		return;

	Effector *e = effectors_[index];
	const string &joint = e->getJoint()->getName();
	vector<Effector *> &es = effectors[joint];

	for(size_t i = es.size(); i --> 0;)
	{
		if(es[i] == e)
		{
			delete e;
			es.erase(es.begin() + i);
			effectors_.erase(effectors_.begin() + index);
			numEffectors--;
			return;
		}
	}
}

//===========================================================================//

void Skeleton::initEffectors(size_t n)
{
	clearEffectors();
	effectors_.resize(n);
}

//===========================================================================//

void Skeleton::clearEffectors()
{
	if(effectors_.size() == 0)
		return;

	for(size_t i = 0; i < effectors_.size(); ++i)
		delete effectors_[i];

	effectors_.clear();
	effectors.clear();
	numEffectors = 0;
}

//===========================================================================//

bool Skeleton::hasEffectors(const std::string &joint)
{
	return effectors.find(joint) != effectors.end();
}

//===========================================================================//

Joint *Skeleton::getRoot()
{
	return root;
}

Joint *Skeleton::getPoseJoint()
{
	return joints[poseJoint];
}

std::vector<int> Skeleton::getPoseJointIndices()
{
	std::vector<int> is(6);

	is[0] = tx;
	is[1] = ty;
	is[2] = tz;

	is[3] = rx;
	is[4] = ry;
	is[5] = rz;

	return is;
}

bool Skeleton::hasPoseJoint()
{
	return !poseJoint.empty();
}

Joint *Skeleton::getScaleJoint()
{
	return joints[scaleJoint];
}

float Skeleton::getScale()
{
	return scaleVal;
}

float Skeleton::getScaleMagnitude()
{
	if(!scaleJoint.empty())
		return initialTransformations[scaleJoint](0,0);
	else
		return 1.0f;
}

//===========================================================================//

Vec3f Skeleton::getEulerAngles()
{
	return Vec3f(current[rx], current[ry], current[rz]);
}

std::vector<Vec3f> Skeleton::getPhalanxEndpoints()
{
	vector<Vec3f> ps;

	map<string, Joint *>::iterator it;

	for(it = joints.begin(); it != joints.end(); ++it)
		ps.push_back(
			Transform3f(it->second->getGlobalTransformation()) * Vec3f(0,1,0) );

	return ps;
}

//===========================================================================//

const std::vector<float>& Skeleton::getCurrentParameters() {
	return current;
}

//===========================================================================//

VectorN Skeleton::get() {
    VectorN theta = VectorN::Zero(num_thetas);
    for (int i = 0; i < num_thetas; ++i) {
        theta[i] = current[i];
    }
    return theta;
}

//===========================================================================//

std::vector<float> Skeleton::getUpdatedParameters(
	const vector<float> &ps,
	const vector<float> &dt,
	bool psDim)
{
	size_t n = psDim ? ps.size() : dt.size();

	std::vector<float> updated(n);

	for(size_t i = 0; i < n; ++i)
		updated[i] = ps[i] + (i < dt.size() ? dt[i] : 0);

	if(poseJoint.empty())
		return updated;

	Vector3f axisX(
			mapping.getJointInfo(rx).axis[0],
			mapping.getJointInfo(rx).axis[1],
			mapping.getJointInfo(rx).axis[2] );
	Vector3f axisY(
			mapping.getJointInfo(ry).axis[0],
			mapping.getJointInfo(ry).axis[1],
			mapping.getJointInfo(ry).axis[2] );
	Vector3f axisZ(
			mapping.getJointInfo(rz).axis[0],
			mapping.getJointInfo(rz).axis[1],
			mapping.getJointInfo(rz).axis[2] );

	Transform3f rX(Quaternionf(AngleAxisf(dt[rx], axisX)));
	Transform3f rY(Quaternionf(AngleAxisf(dt[ry], axisY)));
	Transform3f rZ(Quaternionf(AngleAxisf(dt[rz], axisZ)));

	Mat3f r = (rZ * rX * rY).rotation();

	r = joints[poseJoint]->getRotation() * r;

	Vec3f e = mapping.toEulerAngles(r);

	updated[rx] = e[0];
	updated[ry] = e[1];
	updated[rz] = e[2];

	return updated;
}

//===========================================================================//

void Skeleton::setMapping(const Mapping &map)
{
	mapping = map;
	setPoseJoint(mapping.getPoseJoint(),0,1,2,3,4,5);
	setScaleJoint(mapping.getScaleJoint());
	current.resize(mapping.getNumberOfParameters());
}

//===========================================================================//

Mapping &Skeleton::getMapping()
{
	return mapping;
}

void Skeleton::generateIDs()
{
    size_t numjs = joints_.size();
    bool print_joint_ids = false;
    for(size_t i = 0; i < numjs; ++i)
    {
#define LINEAR_IDEXING
#ifndef LINEAR_IDEXING
        const size_t minID = 0;
        const size_t maxID = 254; ///< 255 is invalid!            
        float x = (float)i / (float)numjs;
        size_t id = minID + x * (maxID - minID);
#else
        size_t id = i;
#endif
        const std::string &name = joints_[i]->getName();
        ids[id] = name;
        ids2[name] = id;
        
        if(print_joint_ids)
            printf("[id=%ld] --> %s\n", id, name.c_str());
    }
}

//===========================================================================//

float Skeleton::getEffectorDistance()
{
    float error = 0;
    
    map< string, vector<Effector *> >::iterator it;
    
    for(it = effectors.begin(); it != effectors.end(); ++it)
    {
        vector<Effector *> es = it->second;
        
        for(size_t i = 0; i < es.size(); ++i)
        {
            const Vec3f s = es[i]->getPosition();
            const Vec3f t = es[i]->getTarget();
            const Vec3f d = t - s;
            
            error += d[0]*d[0] + d[1]*d[1] + d[2]*d[2];
        }
    }
    
    return error;
}

//===========================================================================//

Joint *Skeleton::getJoint(const std::string &joint)
{
    if(joints.find(joint) == joints.end())
        return NULL;
	return joints[joint];
}

std::vector<Effector *> &Skeleton::getEffectors(const std::string &joint)
{
	return effectors[joint];
}

std::vector<Joint *> &Skeleton::getJoints_()
{
	return joints_;
}

std::vector<Effector *> &Skeleton::getEffectors_()
{
	return effectors_;
}

std::map< std::string, Joint *> &Skeleton::getJoints()
{
	return joints;
}

std::map< std::string, std::vector<Effector *> > &Skeleton::getEffectors()
{
	return effectors;
}

int Skeleton::getNumberOfJoints()
{
	return numJoints;
}

int Skeleton::getNumberOfEffectors()
{
	return numEffectors;
}

//===========================================================================//

void Skeleton::setGlobalTranslation(Joint *joint, const Vec3f &target, bool undo)
{
    // get joint's parent and child joints
    Joint *parent = joint->getParent();
    vector<Joint *> &children = joint->getChildren();

    // compute local displacement
    Vec3f source = joint->getGlobalTranslation();
    Vec3f diff = target - source;

    Mat3f rotinv = Mat3f::Identity();
    if(parent && !isStructural(parent))
        rotinv = parent->getGlobalRotation().inverse();
    Vec3f d = rotinv * diff;

    // prepare for back-translation of children
    vector<Vec3f> child_targets(children.size());
    if(joint->hasChildren())
    {
        for(size_t i = 0; i < children.size(); ++i)
            child_targets[i] = children[i]->getGlobalTranslation();
    }

    // move the joint towards the target
    joint->translate(d);

    // move the children back to their original positions
    if(undo && joint->hasChildren())
    {
        for(size_t i = 0; i < children.size(); ++i)
        {
            Vec3f child_source = children[i]->getGlobalTranslation();
            diff = child_targets[i] - child_source;
            d = joint->getGlobalRotation().inverse() * diff;
            children[i]->translate(d);
        }
    }

    // accept changes made to the skeleton
    setInitialTranslations();
}

//===========================================================================//

void Skeleton::translateGlobal(Joint *joint, const Vec3f &t)
{
    setGlobalTranslation(joint, joint->getGlobalTranslation() + t);
}

//===========================================================================//

void Skeleton::alignRotation(Joint *joint, bool wristHack)
{
    // rotate the joint so it lines up with its end position
    if(!joint->hasChildren())
        return;

    // work with rest posture
    vector<float> posture = current;
    reset();

    // end position is average child position
    Vec3f endpos(0,0,0);
    float numcs = 0.0f;
    vector<Joint *> &children = joint->getChildren();
    for(size_t i = 0; i < children.size(); ++i)
    {
        if(wristHack)
        {
			Joint *c = children[i];
            const std::string &n = c->getName();
			if(contains_string(n, "Index") || contains_string(n, "Pinky"))
			{
				endpos += c->getGlobalTranslation();
				numcs  += 1.0f;
			}
        }
        else
        {
            endpos += children[i]->getGlobalTranslation();
            numcs  += 1.0f;
        }
	}
	if(numcs > 0) endpos /= numcs;

    // source and target forward axis
    Vec3f a = Vec3f(0.0f, 1.0f, 0.0f);
    Vec3f g = (endpos - joint->getGlobalTranslation()).normalized();
    Vec3f b = (joint->getGlobalRotation().inverse() * g).normalized();

    // relative rotation
    Quaternionf q = Quaternionf::FromTwoVectors(a,b);
    Mat4f m = Transform3f(q).matrix();

    // prepare for back-translation of children
    vector<Vec3f> child_targets(children.size());
    if(joint->hasChildren())
    {
        for(size_t i = 0; i < children.size(); ++i)
            child_targets[i] = children[i]->getGlobalTranslation();
    }

    // rotate joint (reverse matrix muliplication order)
    joint->transform(m, false);

    // rotate and translate children back
    if(joint->hasChildren())
    {
        Mat4f m_inv = m.inverse();

        for(size_t i = 0; i < children.size(); ++i)
        {
            // undo rotation
            children[i]->transform(m_inv);

            // undo translation
            Vec3f child_source = children[i]->getGlobalTranslation();
            Vec3f diff = child_targets[i] - child_source;
            Vec3f d = joint->getGlobalRotation().inverse() * diff;
            children[i]->translate(d);
        }
    }

    // accept changes made to the skeleton
    setInitialTransformations();
    set(posture);
}

//===========================================================================//

void Skeleton::scaleWidth(float s)
{
    vector<string> js(5);

    js[0] = "HandThumb1";
    js[1] = "HandIndex1";
    js[2] = "HandMiddle1";
    js[3] = "HandRing1";
    js[4] = "HandPinky1";

    Vec3f center = getPoseJoint()->getGlobalTranslation();
    Vec3f axis = getPoseJoint()->getGlobalRotation() * Vec3f(1,0,0);
    axis.normalize();
    float factor = (center - getJoint(js[2])->getGlobalTranslation()).norm();

    for(size_t i = 0; i < js.size(); ++i)
    {
        Joint *j = getJoint(js[i]);
        Vec3f pos = j->getGlobalTranslation();
        Vec3f dir = (pos - center)/factor * s;
        Vec3f offset = axis * (dir.dot(axis));
        Vec3f newpos = pos + offset;
        setGlobalTranslation(j, newpos, false);
        //alignRotation(getPoseJoint(), true);
    }
}

//===========================================================================//

void Skeleton::scaleHeight(float s)
{
    // finger bases

    vector<string> js(5);

    js[0] = "HandThumb1";
    js[1] = "HandIndex1";
    js[2] = "HandMiddle1";
    js[3] = "HandRing1";
    js[4] = "HandPinky1";

    Vec3f center = getPoseJoint()->getGlobalTranslation();
    Vec3f axis = getPoseJoint()->getGlobalRotation() * Vec3f(0,1,0);
    axis.normalize();
    float factor = (center - getJoint(js[2])->getGlobalTranslation()).norm();

    for(size_t i = 0; i < js.size(); ++i)
    {
        Joint *j = getJoint(js[i]);
        Vec3f pos = j->getGlobalTranslation();
        Vec3f dir = (pos - center)/factor * s;
        Vec3f offset = axis * (dir.dot(axis));
        Vec3f newpos = pos + offset;
        setGlobalTranslation(j, newpos, false);
        //alignRotation(getPoseJoint(), true);
    }

    // finger segments

    js.resize(5*3);

    js[ 0] = "HandThumb2";
    js[ 1] = "HandThumb3";
    js[ 2] = "HandThumb4";

    js[ 3] = "HandIndex2";
    js[ 4] = "HandIndex3";
    js[ 5] = "HandIndex4";

    js[ 6] = "HandMiddle2";
    js[ 7] = "HandMiddle3";
    js[ 8] = "HandMiddle4";

    js[ 9] = "HandRing2";
    js[10] = "HandRing3";
    js[11] = "HandRing4";

    js[12] = "HandPinky2";
    js[13] = "HandPinky3";
    js[14] = "HandPinky4";

    for(size_t i = 0; i < js.size(); ++i)
    {
        Joint *j = getJoint(js[i]);
        j->translate(Vec3f(0,1,0)*s/3.0f);
    }
}

//===========================================================================//



//===========================================================================//



//===========================================================================//



//===========================================================================//



//===========================================================================//



//===========================================================================//



//===========================================================================//

void Skeleton::toRightHand()
{
    setMapping(Mapping::rightHand());

	for(Joint *j : getJoints_())
	{
        if(isStructural(j))
            continue;
        
		Mat4f m = Mat4f::Identity();
		m(0,0) = -1.0f;
		j->transform(m);

		m = j->getLocalTransformation();
		m.block(0,0,4,1) = (Mat4f::Identity() * -1.0f) * m.block(0,0,4,1);
		j->setTransformation(m);
	}

	setInitialTransformations();
//	reset();
}

//===========================================================================//

void Skeleton::toRightHand2()
{
    setMapping(Mapping::rightArm2());

	for(Joint *j : getJoints_())
	{
        if(isStructural(j))
            continue;
        
		Mat4f m = Mat4f::Identity();
		m(0,0) = -1.0f;
		j->transform(m);

		m = j->getLocalTransformation();
		m.block(0,0,4,1) = (Mat4f::Identity() * -1.0f) * m.block(0,0,4,1);
		j->setTransformation(m);
	}

	setInitialTransformations();
//	reset();
}

//===========================================================================//

void Skeleton::toRightArm()
{
    setMapping(Mapping::rightArm());

	for(Joint *j : getJoints_())
	{
        if(isStructural(j))
            continue;
        
		Mat4f m = Mat4f::Identity();
		m(0,0) = -1.0f;
		j->transform(m);

		m = j->getLocalTransformation();
		m.block(0,0,4,1) = (Mat4f::Identity() * -1.0f) * m.block(0,0,4,1);
		j->setTransformation(m);
	}

	setInitialTransformations();
//	reset();
}

//===========================================================================//

void Skeleton::toRightArm2()
{
    setMapping(Mapping::rightArm2());

	for(Joint *j : getJoints_())
	{
        if(isStructural(j))
            continue;
        
		Mat4f m = Mat4f::Identity();
		m(0,0) = -1.0f;
		j->transform(m);

		m = j->getLocalTransformation();
		m.block(0,0,4,1) = (Mat4f::Identity() * -1.0f) * m.block(0,0,4,1);
		j->setTransformation(m);
	}

	setInitialTransformations();
//	reset();
}

//===========================================================================//
