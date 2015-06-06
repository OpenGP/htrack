#pragma once
#include "tracker/Types.h"
#include "util/mylogger.h"
#include "tracker/Legacy/geometry/Mapping.h"
#include "tracker/Legacy/geometry/Skeleton.h"
#include "tracker/Legacy/geometry/Cylinders.h"
#include "CustomJointInfo.h"
#include <vector>

class SkeletonSerializer : public Skeleton{
public:
    ///--- Default skeleton position
    float* skeleton_translation = new float[3] {0.0, -70, 400};
    float skeleton_orientation = 0; ///< rotation about "z"
    float skeleton_scale = 0.9f;

private:
    KinematicChain adj; ///< each row contains the joint id of kinematic chain (including itself), terminated by -1
    JointTransformations jinfos_new;

public:
    SkeletonSerializer(Joint *root, const Mapping &mapping);

    const KinematicChain& getKinematicChain(){ return adj; }
    const JointTransformations& getJointTransformations(){ return jinfos_new; }
    

public:
    void update();
    Matrix_3xN jacobian(int joint_id, const Vector3& position);
    void jacobian(Eigen::Matrix<Scalar, 3, num_thetas> &J, Vector3 s, size_t id);
    void reset_hand();
};
