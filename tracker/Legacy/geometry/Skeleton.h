#pragma once

#include <map>
#include <vector>

#include "../math/MathUtils.h"
#include "Joint.h"
#include "Effector.h"
#include "Mapping.h"
#include "tracker/Types.h"

class Skeleton{
public:
    /// @note I assume elsewhere this is signed
    /// @note invalid when set to -1
    typedef Integer JointID;
    
public:
    Skeleton(Joint *root, const Mapping &mapping);
    Skeleton();

    ~Skeleton();

    void move(const std::vector<float> &parameters);
    void set(const std::vector<float> &parameters);
    void set(VectorN parameters);
    void moveEffectors();

    void rotate(const Vec3f &axis, float angle);
    void translate(const Vec3f &translation);
    void transform(const Mat4f &matrix);
    void scale(float value);

    void setRotation(const Mat3f &matrix);
    void setRotation(const Vec3f &axis, float angle);
    void setTranslation(const Vec3f &translation);
    void setTransformation(const Mat4f &matrix);
    void setScale(float value);
    void setScaleValue(float value);

    void scaleWidth(float s);
    void scaleHeight(float s);

public:

    void transformJoints(const std::vector<float> &parameters);

    void setRoot(Joint *root);
    void setForwardAxis(const Vec3f &xyz);

    void setPoseJoint(
            const std::string &name,
            int tx=0, int ty=1, int tz=2,
            int rx=3, int ry=4, int rz=5);

    void setScaleJoint(
            const std::string &name);

    void setInitialTransformation(const std::string &jointName, const Mat4f &m);
    void setInitialTransformations(const aligned_map<std::string, Mat4f>::type &ts);
    void setInitialTransformations();
    void setInitialTranslations();
    void reset(bool reinit = false);
    void resetPosture(bool reinit = false);

    aligned_map<std::string, Mat4f>::type getInitialTransformations();
    Mat4f getInitialTransformation(const std::string &jointName);

    void addJoint(Joint *joint);
    void addEffector(Effector *effector);
    void addEffector(Effector *effector, size_t index);
    void clearJoints();

    void removeEffector(size_t index);
    void initEffectors(size_t n);
    void clearEffectors();
    bool hasEffectors(const std::string &joint);

    Joint *getRoot();
    Joint *getJoint(const std::string &joint);
    Joint *getJoint(int index)
    {
        return joints_[index];
    }

    bool isRoot(Joint *joint)
	{
        return joint == root;
    }

    bool isStructural(Joint *joint)
	{
        return joint == root
            || joint == joints[poseJoint]
            || joint == joints[scaleJoint];
    }

public:

    void setGlobalTranslation(Joint *joint, const Vec3f &target, bool undo = true);
    void translateGlobal(Joint *joint, const Vec3f &target);
    void alignRotation(Joint *joint, bool wristHack = false);

public:

    Joint *getPoseJoint();
    std::vector<int> getPoseJointIndices();
    bool hasPoseJoint();
    Vec3f getEulerAngles();
    std::vector<Vec3f> getPhalanxEndpoints();

    Joint *getScaleJoint();
    float getScale();
    float getScaleMagnitude();

    std::map<std::string, Joint *> &getJoints();
    std::map<std::string, std::vector<Effector *> > &getEffectors();
    std::vector<Effector *> &getEffectors(const std::string &joint);

    std::vector<Joint *> &getJoints_();
    std::vector<Effector *> &getEffectors_();

    int getNumberOfJoints();
    int getNumberOfEffectors();
    float getEffectorDistance();

    const std::vector<float>& getCurrentParameters();
    VectorN get();
    std::vector<float> getUpdatedParameters(
            const std::vector<float> &ps,
            const std::vector<float> &dt,
            bool psDim = false);

    void setMapping(const Mapping &mapping);
    Mapping &getMapping();

public:

    void generateIDs();

    bool hasIDs()
	{
        return !ids.empty() && !ids2.empty();
    }

    std::string getJointName(JointID id)
	{
        assert(id>=0);
        return ids[id];
    }
    
    JointID getID(const std::string &n)
	{
        return ids2[n];
    }

#if 0 
    /// unused
	JointID getID(size_t index)
	{
        size_t minID = 55;
        size_t maxID = 200;
        size_t numjs = joints_.size();
		float x = (float)index / (float)numjs;
		return minID + x * (maxID - minID);
	}
#endif

#if 0 
    /// unused!
	size_t getIndex(JointID id)
	{
        size_t minID = 55;
        size_t maxID = 200;
        size_t numjs = joints_.size();
		float x = (float)(id - minID) / (float)(maxID - minID);
		return x * numjs;
	}
	size_t getIndex(const std::string &n)
	{
		return getIndex(getID(n));
	}
#endif
          
public:

    template <class SkelType>
    static SkelType *leftHand()
    {
        // create basic left hand skeleton

        Joint *root = new Joint("root");
        Joint *pose = new Joint("pose", root);
        Joint *scale = new Joint("scale", pose);

        Joint *Hand = new Joint("Hand", scale);

        Joint *HandThumb1 = new Joint("HandThumb1", Hand);
        Joint *HandThumb2 = new Joint("HandThumb2", HandThumb1);
        Joint *HandThumb3 = new Joint("HandThumb3", HandThumb2);
        Joint *HandThumb4 = new Joint("HandThumb4", HandThumb3);

        Joint *HandPinky1 = new Joint("HandPinky1", Hand);
        Joint *HandPinky2 = new Joint("HandPinky2", HandPinky1);
        Joint *HandPinky3 = new Joint("HandPinky3", HandPinky2);
        Joint *HandPinky4 = new Joint("HandPinky4", HandPinky3);

        Joint *HandRing1 = new Joint("HandRing1", Hand);
        Joint *HandRing2 = new Joint("HandRing2", HandRing1);
        Joint *HandRing3 = new Joint("HandRing3", HandRing2);
        Joint *HandRing4 = new Joint("HandRing4", HandRing3);

        Joint *HandMiddle1 = new Joint("HandMiddle1", Hand);
        Joint *HandMiddle2 = new Joint("HandMiddle2", HandMiddle1);
        Joint *HandMiddle3 = new Joint("HandMiddle3", HandMiddle2);
        Joint *HandMiddle4 = new Joint("HandMiddle4", HandMiddle3);

        Joint *HandIndex1 = new Joint("HandIndex1", Hand);
        Joint *HandIndex2 = new Joint("HandIndex2", HandIndex1);
        Joint *HandIndex3 = new Joint("HandIndex3", HandIndex2);
        Joint *HandIndex4 = new Joint("HandIndex4", HandIndex3);

        SkelType *skeleton = new SkelType(root, Mapping::leftHand());

        // update joint transformations with reasonable values

        // segment length ratios roughly based on:
        //  "Proportions of Hand Segments"
        //  by Alexander Buryanov and Viktor Kotiuk
        //  in International journal of morphology, 2010

        float p = 0.75f;
        float r = 0.95f;
        float m = 1.00f;
        float i = 0.95f;

        map<string, Joint *> &joints = skeleton->getJoints();

        // initial rotations

        joints["HandThumb1"]->rotate(Vec3f(0.205597, 0.659382, -0.723149), 1.09729);
        joints["HandThumb2"]->rotate(Vec3f(-0.91756, -0.222654, -0.329407), 0.223523);
        joints["HandThumb3"]->rotate(Vec3f(1.53041e-15, 1, -6.19152e-16), 0.0523502);
        joints["HandThumb4"]->rotate(Vec3f(1.53041e-15, 1, -6.19152e-16), 0.0523502);

        joints["HandPinky1"]->rotate(Vec3f(0.0917447, 0.00561026, 0.995767), 0.468236);
        joints["HandPinky2"]->rotate(Vec3f(0.305667, -0.0629561, -0.950055), 0.0585836);
        joints["HandPinky3"]->rotate(Vec3f(-0.99938, -0.0016505, -0.0351619), 0.0564697);
        joints["HandPinky4"]->rotate(Vec3f(-0.99938, -0.0016505, -0.0351619), 0.0564697);

        joints["HandRing1"]->rotate(Vec3f(-0.179293, -0.0674548, 0.98148), 0.228987);
        joints["HandRing2"]->rotate(Vec3f(-0.989645, -0.00584321, 0.143415), 0.0228087);
        joints["HandRing3"]->rotate(Vec3f(-0.0726904, -0.0533319, 0.995928), 0.0409756);
        joints["HandRing4"]->rotate(Vec3f(-0.0726904, -0.0533319, 0.995928), 0.0409756);

        joints["HandMiddle1"]->rotate(Vec3f(-0.986337, -0.0844559, 0.141442), 0.183442);
        joints["HandMiddle2"]->rotate(Vec3f(1, -0.000106962, 0.000958682), 0.118737);
        joints["HandMiddle3"]->rotate(Vec3f(1, 0, 0), 0);
        joints["HandMiddle4"]->rotate(Vec3f(1, 0, 0), 0);

        joints["HandIndex1"]->rotate(Vec3f(-0.476729, -0.0362517, -0.878303), 0.194492);
        joints["HandIndex2"]->rotate(Vec3f(0.912837, -0.019366, 0.407864), 0.0796325);
        joints["HandIndex3"]->rotate(Vec3f(1, 0, 0), 0);
        joints["HandIndex4"]->rotate(Vec3f(1, 0, 0), 0);

        // initial translations

        joints["HandThumb1"]->setTranslation(1.0 * Vec3f(20.0, 10.0f,-10.0));
        joints["HandThumb2"]->setTranslation(0.8 * Vec3f(   0, 50.0f,    0));
        joints["HandThumb3"]->setTranslation(0.8 * Vec3f(   0, 30.0f,    0));
        joints["HandThumb4"]->setTranslation(0.8 * Vec3f(   0, 20.0f,    0));

        joints["HandPinky1"]->setTranslation(1.0 * Vec3f(-30.0,     80.0f, 0));
        joints["HandPinky2"]->setTranslation(0.8 * Vec3f(    0, p * 50.0f, 0));
        joints["HandPinky3"]->setTranslation(0.8 * Vec3f(    0, p * 30.0f, 0));
        joints["HandPinky4"]->setTranslation(0.8 * Vec3f(    0, p * 20.0f, 0));

        joints["HandRing1"]->setTranslation(1.0 * Vec3f(-10.0,     80.0f, 0));
        joints["HandRing2"]->setTranslation(0.8 * Vec3f(    0, r * 50.0f, 0));
        joints["HandRing3"]->setTranslation(0.8 * Vec3f(    0, r * 30.0f, 0));
        joints["HandRing4"]->setTranslation(0.8 * Vec3f(    0, r * 20.0f, 0));

        joints["HandMiddle1"]->setTranslation(1.0 * Vec3f(10.0,     80.0f, 0));
        joints["HandMiddle2"]->setTranslation(0.8 * Vec3f(   0, m * 50.0f, 0));
        joints["HandMiddle3"]->setTranslation(0.8 * Vec3f(   0, m * 30.0f, 0));
        joints["HandMiddle4"]->setTranslation(0.8 * Vec3f(   0, m * 20.0f, 0));

        joints["HandIndex1"]->setTranslation(1.0 * Vec3f(30.0,     80.0f, 0));
        joints["HandIndex2"]->setTranslation(0.8 * Vec3f(   0, i * 50.0f, 0));
        joints["HandIndex3"]->setTranslation(0.8 * Vec3f(   0, i * 30.0f, 0));
        joints["HandIndex4"]->setTranslation(0.8 * Vec3f(   0, i * 20.0f, 0));

        skeleton->setInitialTransformations();
        skeleton->reset();

        return skeleton;
    }

    template <class SkelType>
    static SkelType *leftHand2()
    {
        SkelType *skeleton = Skeleton::leftHand<SkelType>();
        skeleton->setMapping(Mapping::leftArm2());
        return skeleton;
    }

    template <class SkelType>
    static SkelType *leftArm()
    {
        // create basic left hand skeleton with forearm

        Joint *root = new Joint("root");
        Joint *pose = new Joint("pose", root);
        Joint *scale = new Joint("scale", pose);

        Joint *Hand = new Joint("Hand", scale);

        Joint *HandThumb1 = new Joint("HandThumb1", Hand);
        Joint *HandThumb2 = new Joint("HandThumb2", HandThumb1);
        Joint *HandThumb3 = new Joint("HandThumb3", HandThumb2);
        Joint *HandThumb4 = new Joint("HandThumb4", HandThumb3);
        (void) HandThumb4; // suppress unused warning

        Joint *HandPinky1 = new Joint("HandPinky1", Hand);
        Joint *HandPinky2 = new Joint("HandPinky2", HandPinky1);
        Joint *HandPinky3 = new Joint("HandPinky3", HandPinky2);
        Joint *HandPinky4 = new Joint("HandPinky4", HandPinky3);
        (void) HandPinky4; // suppress unused warning

        Joint *HandRing1 = new Joint("HandRing1", Hand);
        Joint *HandRing2 = new Joint("HandRing2", HandRing1);
        Joint *HandRing3 = new Joint("HandRing3", HandRing2);
        Joint *HandRing4 = new Joint("HandRing4", HandRing3);
        (void) HandRing4; // suppress unused warning

        Joint *HandMiddle1 = new Joint("HandMiddle1", Hand);
        Joint *HandMiddle2 = new Joint("HandMiddle2", HandMiddle1);
        Joint *HandMiddle3 = new Joint("HandMiddle3", HandMiddle2);
        Joint *HandMiddle4 = new Joint("HandMiddle4", HandMiddle3);
        (void) HandMiddle4; // suppress unused warning

        Joint *HandIndex1 = new Joint("HandIndex1", Hand);
        Joint *HandIndex2 = new Joint("HandIndex2", HandIndex1);
        Joint *HandIndex3 = new Joint("HandIndex3", HandIndex2);
        Joint *HandIndex4 = new Joint("HandIndex4", HandIndex3);
        (void) HandIndex4; // suppress unused warning

        Joint *HandForearm = new Joint("HandForearm", Hand);
        Joint *HandForearm4 = new Joint("HandForearm4", HandForearm);
        (void) HandForearm4; // suppress unused warning

        SkelType *skeleton = new SkelType(root, Mapping::leftArm());

        // update joint transformations with reasonable values

        // segment length ratios roughly based on:
        //  "Proportions of Hand Segments"
        //  by Alexander Buryanov and Viktor Kotiuk
        //  in International journal of morphology, 2010

        float p = 0.80f;
        float r = 0.95f;
        float m = 1.00f;
        float i = 0.95f;

        map<string, Joint *> &joints = skeleton->getJoints();

        // forearm stuff

        joints["HandForearm"]->rotate(Vec3f(0,0,1), M_PI);
        joints["HandForearm4"]->setTranslation(Vec3f(0.0f, 60.0f, 0.0f));

        // initial rotations

        joints["HandThumb1"]->rotate(Vec3f(0.205597, 0.659382, -0.723149), 1.09729);
        joints["HandThumb2"]->rotate(Vec3f(-0.91756, -0.222654, -0.329407), 0.223523);
        joints["HandThumb3"]->rotate(Vec3f(1.53041e-15, 1, -6.19152e-16), 0.0523502);
        joints["HandThumb4"]->rotate(Vec3f(1.53041e-15, 1, -6.19152e-16), 0.0523502);

        joints["HandPinky1"]->rotate(Vec3f(0.0917447, 0.00561026, 0.995767), 0.468236);
        joints["HandPinky2"]->rotate(Vec3f(0.305667, -0.0629561, -0.950055), 0.0585836);
        joints["HandPinky3"]->rotate(Vec3f(-0.99938, -0.0016505, -0.0351619), 0.0564697);
        joints["HandPinky4"]->rotate(Vec3f(-0.99938, -0.0016505, -0.0351619), 0.0564697);

        joints["HandRing1"]->rotate(Vec3f(-0.179293, -0.0674548, 0.98148), 0.228987);
        joints["HandRing2"]->rotate(Vec3f(-0.989645, -0.00584321, 0.143415), 0.0228087);
        joints["HandRing3"]->rotate(Vec3f(-0.0726904, -0.0533319, 0.995928), 0.0409756);
        joints["HandRing4"]->rotate(Vec3f(-0.0726904, -0.0533319, 0.995928), 0.0409756);

        joints["HandMiddle1"]->rotate(Vec3f(-0.986337, -0.0844559, 0.141442), 0.183442);
        joints["HandMiddle2"]->rotate(Vec3f(1, -0.000106962, 0.000958682), 0.118737);
        joints["HandMiddle3"]->rotate(Vec3f(1, 0, 0), 0);
        joints["HandMiddle4"]->rotate(Vec3f(1, 0, 0), 0);

        joints["HandIndex1"]->rotate(Vec3f(-0.476729, -0.0362517, -0.878303), 0.194492);
        joints["HandIndex2"]->rotate(Vec3f(0.912837, -0.019366, 0.407864), 0.0796325);
        joints["HandIndex3"]->rotate(Vec3f(1, 0, 0), 0);
        joints["HandIndex4"]->rotate(Vec3f(1, 0, 0), 0);

        // initial translations

        joints["HandThumb1"]->setTranslation(1.0 * Vec3f(20.0, 10.0f,-10.0));
        joints["HandThumb2"]->setTranslation(0.8 * Vec3f(   0, 50.0f,    0));
        joints["HandThumb3"]->setTranslation(0.8 * Vec3f(   0, 30.0f,    0));
        joints["HandThumb4"]->setTranslation(0.8 * Vec3f(   0, 20.0f,    0));

        joints["HandPinky1"]->setTranslation(1.0 * Vec3f(-30.0,     80.0f, 0));
        joints["HandPinky2"]->setTranslation(0.8 * Vec3f(    0, p * 50.0f, 0));
        joints["HandPinky3"]->setTranslation(0.8 * Vec3f(    0, p * 30.0f, 0));
        joints["HandPinky4"]->setTranslation(0.8 * Vec3f(    0, p * 20.0f, 0));

        joints["HandRing1"]->setTranslation(1.0 * Vec3f(-10.0,     80.0f, 0));
        joints["HandRing2"]->setTranslation(0.8 * Vec3f(    0, r * 50.0f, 0));
        joints["HandRing3"]->setTranslation(0.8 * Vec3f(    0, r * 30.0f, 0));
        joints["HandRing4"]->setTranslation(0.8 * Vec3f(    0, r * 20.0f, 0));

        joints["HandMiddle1"]->setTranslation(1.0 * Vec3f(10.0,     80.0f, 0));
        joints["HandMiddle2"]->setTranslation(0.8 * Vec3f(   0, m * 50.0f, 0));
        joints["HandMiddle3"]->setTranslation(0.8 * Vec3f(   0, m * 30.0f, 0));
        joints["HandMiddle4"]->setTranslation(0.8 * Vec3f(   0, m * 20.0f, 0));

        joints["HandIndex1"]->setTranslation(1.0 * Vec3f(30.0,     80.0f, 0));
        joints["HandIndex2"]->setTranslation(0.8 * Vec3f(   0, i * 50.0f, 0));
        joints["HandIndex3"]->setTranslation(0.8 * Vec3f(   0, i * 30.0f, 0));
        joints["HandIndex4"]->setTranslation(0.8 * Vec3f(   0, i * 20.0f, 0));

        skeleton->setInitialTransformations();
        skeleton->reset();

        return skeleton;
    }

    template <class SkelType>
    static SkelType *leftArm2()
    {
        SkelType *skeleton = Skeleton::leftArm<SkelType>();
        skeleton->setMapping(Mapping::leftArm2());
        return skeleton;
    }

    template <class SkelType>
    static SkelType *rightHand()
    {
        SkelType *skeleton = Skeleton::leftHand<SkelType>();
        skeleton->setMapping(Mapping::rightHand());

        for(Joint *j : skeleton->getJoints_())
        {
            Mat4f m = Mat4f::Identity();
            m(0,0) = -1.0f;
            j->transform(m);

            m = j->getLocalTransformation();
            m.block(0,0,4,1) = (Mat4f::Identity() * -1.0f) * m.block(0,0,4,1);
            j->setTransformation(m);
        }

        skeleton->setInitialTransformations();
        skeleton->reset();

        return skeleton;
    }

    template <class SkelType>
    static SkelType* rightHand2()
    {
        SkelType *skeleton = Skeleton::rightHand<SkelType>();
        skeleton->setMapping(Mapping::rightArm2());
        return skeleton;
    }

    template <class SkelType>
    static SkelType *rightArm()
    {
        SkelType *skeleton = Skeleton::leftArm<SkelType>();
        skeleton->setMapping(Mapping::rightArm());

        for(Joint *j : skeleton->getJoints_())
        {
            Mat4f m = Mat4f::Identity();
            m(0,0) = -1.0f;
            j->transform(m);

            m = j->getLocalTransformation();
            m.block(0,0,4,1) = (Mat4f::Identity() * -1.0f) * m.block(0,0,4,1);
            j->setTransformation(m);
        }

        skeleton->setInitialTransformations();
        skeleton->reset();

        return skeleton;
    }

    template <class SkelType>
    static Skeleton *rightArm2()
    {
        Skeleton *skeleton = Skeleton::rightArm<SkelType>();
        skeleton->setMapping(Mapping::rightArm2());
        return skeleton;
    }
    
    void toRightHand();
    void toRightHand2();
    void toRightArm();
    void toRightArm2();

protected:

    Mapping mapping;

    Joint *root;

    std::string poseJoint;
    std::string scaleJoint;

    int tx, ty, tz, rx, ry, rz;
    float scaleVal;

    std::map<std::string, Joint *> joints;
    std::map<std::string, std::vector<Effector *> > effectors;

    std::vector<Joint *> joints_;
    std::vector<Effector *> effectors_;

    int numJoints;
    int numEffectors;

    Vec3f forward;

    std::vector<float> current;

    aligned_map<std::string, Mat4f>::type initialTransformations;

    // joint ids
    std::map<size_t, std::string> ids;  /// id => name
    std::map<std::string, size_t> ids2; /// name => id

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
