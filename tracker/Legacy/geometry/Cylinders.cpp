#include "Cylinders.h"
#include <vector>

using namespace std;

//===========================================================================//

float dist_point_linesegment(const Vec3f& _p,
							 const Vec3f& _v0,
							 const Vec3f& _v1,
							 Vec3f&	   _nearest_point)
{
	Vec3f d1(_p - _v0);
	Vec3f d2(_v1 - _v0);
	Vec3f min_v(_v0);

	float t = d2.dot(d2);

	if(t > numeric_limits<float>::min())
	{
		t = d1.dot(d2) / t;
		if      (t > 1.0) d1 = _p - (min_v = _v1);
		else if (t > 0.0) d1 = _p - (min_v = _v0 + d2*t);
	}

	_nearest_point = min_v;
	return d1.norm();
}

//===========================================================================//

float Segment::distanceTo(const Vec3f &point, Vec3f &np, Vec3f &n)
{
#if 0
	Mat3f R  = Transform3f(joint->getGlobalRotation()).rotation();
	Mat4f T  = joint->getGlobalTransformation(); T.block(0,0,3,3) = R;
	Mat4f Ti = T.inverse();

    // compute projection on center line segment
    Vec3f start = joint->getGlobalTranslation();
    Vec3f end = joint->getEndPosition();
    dist_point_linesegment(point, start, end, np);

    // interpolate start/end radius
    float le = (end - start).norm();
    float lp = (np - start).norm();
    float t  = lp / le;
    float r  = t * radius2   + (1 - t) * radius1;
    float ry = t * radius2_y + (1 - t) * radius1_y;
    float ratio = ry/r;

    // scale from elliptical to circular
    Vec3f p = Transform3f(Ti) * point;
    p[0] *= ratio;
    p = Transform3f(T) * p;
    float d = (p - np).norm();

    // project onto cylindroid surface
    n = (p - np).normalized();
    np += n * r * ratio;

    // only allow front-facing points
    //glm::vec3 view(0,0,-1);
    //backface_check(point, p, n, view, start, end, r1, r2, r1y, r2y, l);

    // unscale from circular to elliptical
    np = Transform3f(Ti) * np;
    np[0] /= ratio;
    np = Transform3f(T) * np;

    // compute correct normal at this point
    Vec3f nrm = Transform3f(Ti) * n;
    nrm[0] *= ratio;
    n = (Transform3f(T) * nrm).normalized();

    return d;
#else
	Mat3f R  = Transform3f(joint->getGlobalRotation()).rotation();
	Mat4f T  = joint->getGlobalTransformation(); T.block(0,0,3,3) = R;
	Mat4f Ti = T.inverse();

	Vec3f p(Transform3f(Ti) * point);
	Vec3f a(0,0,0);
	Vec3f b(0,length,0);

    float ratio = (radius1_y/radius1);
	p[0] *= ratio;

	float d = dist_point_linesegment(p, a, b, np);

	float t = np[1] / length;
	float r = t * radius2 + (1 - t) * radius1;

    n = (p - np).normalized();
	np += n * r * ratio;

	np[0] /= ratio;
	np = Transform3f(T) * np;
    n[0] *= ratio;
    n = (Transform3f(T) * n).normalized();

	//return d;
    return (point - np).norm();
#endif
}

//===========================================================================//

Cylinders::Cylinders(Skeleton *skeleton, bool hand)
{
	this->skeleton = skeleton;

	const vector<Joint *> &joints = skeleton->getJoints_();

	for(size_t i = 0; i < joints.size(); ++i)
	{
		Joint *j = joints[i];

		if(!skeleton->isStructural(j) && j->hasChildren())
		{
			segments.push_back(Segment(j));
            ids[skeleton->getID(j->getName())] = segments.size()-1;
		}
	}

	scale = skeleton->getScaleMagnitude();

	if(hand) handSegments();

    serial_ptr = NULL;
}

//===========================================================================//

Cylinders::~Cylinders()
{
    if(serial_ptr)
        delete[] serial_ptr;
}

//===========================================================================//

struct JointDistance
{
	float  distance;
	Vec3f  point;
    Vec3f  normal;
	string joint;

	JointDistance()
		: distance(1e+37), point(0,0,0), normal(0,0,0)
	{
	}

	JointDistance(float distance, const Vec3f &point, const Vec3f &normal, const string &joint)
		: distance(distance), point(point), normal(normal), joint(joint)
	{
	}

	bool operator()(const JointDistance &a, const JointDistance &b)
	{
		return a.distance < b.distance;
	}
};

//===========================================================================//

float Cylinders::nearest(const Vec3f &point,
	Vec3f &nearestPoint, Vec3f &normal,
    string &nearestJoint)
{
	vector<JointDistance> ds;

	for(size_t i = 0; i < segments.size(); ++i)
	{
		Vec3f p, n;
		float d = segments[i].distanceTo(point, p, n);
		const string &j = segments[i].joint->getName();
		ds.push_back(JointDistance(d, p, n, j));
	}

	JointDistance nn = *std::min_element(ds.begin(), ds.end(), JointDistance());

	nearestPoint = nn.point;
	nearestJoint = nn.joint;
    normal = nn.normal;

	return nn.distance;
}

//===========================================================================//

bool compareSuffix(const std::string &fullString, const std::string &ending)
{
	if(fullString.length() >= ending.length())
	{
		return ( 0 == fullString.compare(
			fullString.length() - ending.length(),
			ending.length(), ending) );
	}
	else return false;
}

//===========================================================================//

void Cylinders::handSegments()
{    
    float r0 = 8.0;
#if 0
    float hand_width_base = 2;
    float hand_width_top = 2;
    float hand_thickness_base = 2;
    float hand_thickness_top = 2;
#else
    float hand_width_base = 4.5;
    float hand_width_top = 5.2;
    float hand_thickness_base = 1.35;
    float hand_thickness_top = 1.35;
#endif

	for(size_t i = 0; i < segments.size(); ++i)
	{
        float r1, r2, r1_y, r2_y, l;
        r1 = r2 = r1_y = r2_y = l = r0;

		const string &n = segments[i].joint->getName();
		const float s = skeleton->getScaleMagnitude();

        /// Length of a cylinders is simply the distance between joint point and 
        /// (average) joint position of all the children of a joint
		l = (segments[i].joint->getGlobalTranslation() - segments[i].joint->getEndPosition()).norm() / s;

		if(compareSuffix(n, "Hand"))
		{
			r1   = r0 * hand_width_base;
            r2   = r0 * hand_width_top;
            r1_y = r0 * hand_thickness_base;
            r2_y = r0 * hand_thickness_top;
		}
		else if(compareSuffix(n, "Thumb1"))
		{
			r1 = r0 * 1.7f;
			r2 = r0 * 1.0f;
            r1_y = r1;
            r2_y = r2;
		}
		else if(compareSuffix(n, "Thumb2"))
		{
			r1 = r0 * 1.0f;
			r2 = r0 * 0.9f;
            r1_y = r1;
            r2_y = r2;
		}
		else if(compareSuffix(n, "Thumb3"))
		{
			r1 = r0 * 0.9f;
			r2 = r0 * 0.8f;
            r1_y = r1;
            r2_y = r2;
		}
		else if(compareSuffix(n, "1"))
		{
			r1 = r0 * 1.0f;
			r2 = r0 * 0.9f;
            r1_y = r1;
            r2_y = r2;
		}
		else if(compareSuffix(n, "2"))
		{
			r1 = r0 * 0.9f;
			r2 = r0 * 0.8f;
            r1_y = r1;
            r2_y = r2;
		}
		else if(compareSuffix(n, "3"))
		{
			r1 = r0 * 0.8f;
			r2 = r0 * 0.8f;
            r1_y = r1;
            r2_y = r2;
		}
		else if(compareSuffix(n, "Forearm"))
		{
			r1 = r0 * 4.0f;
			r2 = r0 * 4.0f;
            r1_y = r1*1.2f / 4.0f;
            r2_y = r2*1.2f / 4.0f;
		}

		segments[i].radius1 = r1;
		segments[i].radius2 = r2;
		segments[i].radius1_y = r1_y;
        segments[i].radius2_y = r2_y;
		segments[i].length  = l;
	}
}

//===========================================================================//

void Cylinders::recomputeLengths()
{
	for(size_t i = 0; i < segments.size(); ++i)
	{
		float s = skeleton->getScaleMagnitude();
		float l = (segments[i].joint->getGlobalTranslation()
                -  segments[i].joint->getEndPosition(false)).norm() / s;
		segments[i].length = l;
    }
}

//===========================================================================//

float *Cylinders::serialize()
{
    int cols = 44; // number of segment values
    int rows = 17; // number of segment joints
    int n = rows * cols;
    float scale = skeleton->getScaleMagnitude();

    assert(segments.size() <= rows);

    if(!serial_ptr)
        serial_ptr = new float[n];

    for(int i = 0; i < n; ++i)
        serial_ptr[i] = -1.0f;

    for(size_t i = 0; i < segments.size(); ++i)
    {
        size_t k = 0;

        Segment &s = segments[i];

        // joint id
        int id = skeleton->getID(s.joint->getName());
        serial_ptr[i*cols + k++] = (float)id;

        // start position
        Vec3f start = s.joint->getGlobalTranslation();
        serial_ptr[i*cols + k++] = start[0];
        serial_ptr[i*cols + k++] = start[1];
        serial_ptr[i*cols + k++] = start[2];

        // printf("start: %f %f %f\n", start[0], start[1], start[2]);
        assert( !isnan(start[0]) );
        assert( !isnan(start[1]) );
        assert( !isnan(start[2]) );

        // end position
        Vec3f end = s.joint->getEndPosition();
        serial_ptr[i*cols + k++] = end[0];
        serial_ptr[i*cols + k++] = end[1];
        serial_ptr[i*cols + k++] = end[2];
        assert( !isnan(end[0]) );
        assert( !isnan(end[1]) );
        assert( !isnan(end[2]) );

        if( isnan(start[0]) || isnan(end[0]) )
            std::cerr << "!!!WARNING Cylinders::serialize() => NaN" << std::endl;

        // segment parameters (scaled to global)
        serial_ptr[i*cols + k++] = scale * s.radius1;
        serial_ptr[i*cols + k++] = scale * s.radius2;
        serial_ptr[i*cols + k++] = scale * s.radius1_y;
        serial_ptr[i*cols + k++] = scale * s.radius2_y;
        serial_ptr[i*cols + k++] = scale * s.length;

        assert( !isnan(s.radius1) );
        assert( !isnan(s.radius2) );
        assert( !isnan(s.radius1_y) );
        assert( !isnan(s.radius2_y) );
        assert( !isnan(s.length) );

        // joint transform and inverse
        Mat3f R  = Transform3f(s.joint->getGlobalRotation()).rotation();
        Mat4f T  = s.joint->getGlobalTransformation(); T.block(0,0,3,3) = R;
        Mat4f Ti = T.inverse();
        std::copy( T.data(),  T.data() + 16, serial_ptr + i*cols + k);
        std::copy(Ti.data(), Ti.data() + 16, serial_ptr + i*cols + k + 16);
    }

    return serial_ptr;
}

//===========================================================================//

