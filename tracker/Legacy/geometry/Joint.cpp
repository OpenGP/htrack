#include "Joint.h"

#include <Eigen/Geometry>

#include "../util/Util.h"

using namespace Eigen;

Joint::Joint(
	const std::string &name,
	Joint *parent,
	const Vec3f &translation)
{
	this->name = name;
	this->parent = parent;

	children = std::vector<Joint *>();

	global = local = Transform3f(
		Translation3f(translation)).matrix();

	if(this->parent != NULL)
	{
		this->parent->addChild(this);
	}
	else
	{
		update();
	}
}

void Joint::update()
{
	if(parent != NULL)
	{
		global = parent->getGlobalTransformation() * local;
	}
	else
	{
		global = local;
	}

	std::vector<Joint *>::iterator c;

	for(c = children.begin(); c != children.end(); ++c)
	{
		(*c)->update();
	}
}

void Joint::addChild(Joint *child)
{
	children.push_back(child);
	child->update();
}

void Joint::setParent(Joint *parent)
{
	this->parent = parent;
}

void Joint::setChildren(const std::vector<Joint *> &children)
{
	this->children = children;
}

void Joint::setTranslation(const Vec3f &t)
{
	local(0,3) = t[0];
	local(1,3) = t[1];
	local(2,3) = t[2];

	update();
}

void Joint::setRotation(const Vec3f &axis, float angle)
{
	Vec3f t = getTranslation();

	local = Transform3f(
		AngleAxisf(angle, axis)).matrix();

	setTranslation(t);
}

void Joint::setRotation(const Mat3f &matrix)
{
	Vec3f t = getTranslation();

	local = Transform3f(matrix).matrix();

	setTranslation(t);
}

void Joint::setTransformation(const Mat4f &matrix)
{
	local = matrix;
	update();
}

void Joint::translate(const Vec3f &t)
{
	local(0,3) += t[0];
	local(1,3) += t[1];
	local(2,3) += t[2];

	update();
}

void Joint::rotate(const Vec3f &axis, float angle)
{
	local = local * Transform3f(
		AngleAxisf(angle, axis)).matrix();

	update();
}

void Joint::transform(const Mat4f &matrix, bool order)
{
    if(order)
        local = matrix * local;
    else
        local = local * matrix;

	update();
}

std::string Joint::getName()
{
	return name;
}

Vec3f Joint::getTranslation()
{
	return Vec3f(local(0,3), local(1,3), local(2,3));
}

Vec3f Joint::getGlobalTranslation()
{
	return Vec3f(global(0,3), global(1,3), global(2,3));
}

Mat3f Joint::getRotation()
{
	return Mat3f(local.topLeftCorner(3,3)); // with scaling
}

Mat3f Joint::getGlobalRotation()
{
	return Mat3f(global.topLeftCorner(3,3)); // with scaling
}

Mat4f &Joint::getLocalTransformation()
{
	return local;
}

Mat4f &Joint::getGlobalTransformation()
{
	return global;
}

Joint *Joint::getParent()
{
	return parent;
}

std::vector<Joint *> &Joint::getChildren()
{
	return children;
}

Vec3f Joint::getEndPosition(bool force_y)
{
	Vec3f t = getGlobalTranslation();
    Vec3f e = Vec3f::Zero();

	if(!hasChildren())
		return t;

	float l = 0;
	float n = 0;

	bool w = children.size() > 1;

	for(size_t i = 0; i < children.size(); ++i)
	{
		// special case for wrist joint
		if(w && children[i]->hasChildren())
		{
			Joint *c = children[i]->getChildren()[0];
            const std::string &name = c->getName();

            if(force_y)
            {
                // average all fingers for length
                if( contains_string(name, "Index")
                 || contains_string(name, "Middle")
                 || contains_string(name, "Ring")
                 || contains_string(name, "Pinky") )
                {
                    float magic = 315.519f / 4.0f - 338.789f / 5.0f;
                    l += (children[i]->getGlobalTranslation() - t).norm() - magic;
                    n += 1.0f;
                }
            }
            else
            {
                // average only outer fingers for direction
                if( contains_string(name, "Index")
                 || contains_string(name, "Pinky"))
                {
                    float magic = 315.519f / 8.0f - 338.789f / 10.0f;
                    Vec3f dir = children[i]->getGlobalTranslation() - getGlobalTranslation();
                    Vec3f vec = dir.normalized() * (dir.norm() - magic);
                    Vec3f pos = getGlobalTranslation() + vec;
                    e += pos;
                    //e += children[i]->getGlobalTranslation();
                    n += 1.0f;
                }
            }
		}
		// not wrist, standard behaviour
		else if(!w)
		{
            if(force_y)
                l += (children[i]->getGlobalTranslation() - t).norm();
            else
                e += children[i]->getGlobalTranslation();

            n += 1.0f;
		}
	}

	if(n == 0)
		return t;

    if(force_y)
    {
        l /= n;
        Mat3f r = Transform3f(getGlobalTransformation()).rotation();
        return t + r * Vec3f(0,l,0); // hard-coded y-axis as forward axis!
    }
    else
    {
        e /= n;
        return e;
    }
}

