#include "Effector.h"

Effector::Effector(
		Joint *joint,
		const Vec3f &position,
		const Vec3f &target,
		const Vec3f &positionNormal,
		const Vec3f &targetNormal)
{
	this->joint = joint;
	this->position = position;
	this->target = target;

	this->positionNormal = positionNormal;
	this->targetNormal = targetNormal;

	Vec3f dir = position - joint->getGlobalTranslation();
	this->offset = joint->getGlobalRotation().inverse() * dir;

	ratio = 1.0f;

	if(joint->getChildren().size())
	{
		const Vec3f &t1 = joint->getGlobalTranslation();
		const Vec3f &t2 = joint->getChildren()[0]->getGlobalTranslation();

		float l = (t2 - t1).norm();
		float d = offset[1];

		ratio = d / l;
	}
}

Joint *Effector::getJoint()
{
	return joint;
}

Vec3f Effector::getPosition()
{
	return position;
}

Vec3f Effector::getTarget()
{
	return target;
}

Vec3f Effector::getOffset()
{
	return offset;
}

Vec3f Effector::getPositionNormal()
{
	return positionNormal;
}

Vec3f Effector::getTargetNormal()
{
	return targetNormal;
}

void Effector::setPosition(const Vec3f &p)
{
	position = p;
}

void Effector::setTarget(const Vec3f &t)
{
	target = t;
}

void Effector::setOffset(const Vec3f &o)
{
	offset = o;
}

void Effector::setPositionNormal(const Vec3f &pn)
{
	positionNormal = pn;
}

void Effector::setTargetNormal(const Vec3f &tn)
{
	targetNormal = tn;
}

