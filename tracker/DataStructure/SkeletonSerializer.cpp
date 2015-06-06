#include "SkeletonSerializer.h"

SkeletonSerializer::SkeletonSerializer(Joint *root, const Mapping &mapping) : Skeleton(root, mapping){
    ///--- Allocate memory
    adj.resize(joints_.size());
}

void SkeletonSerializer::update(){
    //TIMED_SCOPE(timer, "SkeletonSerializer::update()");

    ///--- Cache the AxisInfo kinematic chain (indexes elements in this->jinfos_new)
    ///    @todo this could be cached, as the chain doesn't change at runtime
    {
        clear_kinematic(adj);

        ///--- Percolate along chain
        for (int i = 0; i < joints_.size(); ++i) {
            Joint joint = *joints_[i]; ///< IMPERATIVE it's a value copy
            int joint_id = this->getID(joint.getName());
            // LOG(INFO) << "\n\nfilling row #" << i << joint_id;
            int colid = 0;
            while(joint.getParent() != NULL){
                // LOG(INFO) << "entering" << joint.getName() << " joint_id" << skeleton->getID(joint.getName());
                const vector<AxisInfo> &infos = mapping.getAxisInfos(joint.getName());
                if(infos.size())
                    for(AxisInfo axis: infos){
                        // LOG(INFO) << "visiting" << axis.index;
                        adj[joint_id].data[colid++] = axis.index;
                    }
                joint = *joint.getParent();
                // if(joint.getParent()==NULL) LOG(INFO) << "Termination!";
            }
        }
        // std::ofstream("kin_chain_1.txt") << adj << endl;
    }

    ///--- Cache the JointInfo content
    jinfos_new.resize(mapping.jointInfos.size());
    for(int jointinfo_id=0; jointinfo_id<mapping.jointInfos.size(); jointinfo_id++){
        JointInfo& jinfo = mapping.jointInfos[jointinfo_id];
        Joint* joint = this->getJoint(jinfo.joint);
        Eigen::Map<Vector3>(jinfos_new[jointinfo_id].axis) = jinfo.axis;
        jinfos_new[jointinfo_id].type = jinfo.type;
        jinfos_new[jointinfo_id].index = jinfo.index;
        if(joint!=NULL)
            switch(jinfo.type){
            case TRANSLATION_AXIS:
            {
                Matrix3 mat = joint->getParent()->getGlobalRotation();
                Eigen::Map<Matrix4>( jinfos_new[jointinfo_id].mat ) = Transform3f(mat).matrix();
                break;
            }
            case ROTATION_AXIS:
            default:
            {
                Matrix4 mat = joint->getGlobalTransformation();
                Eigen::Map<Matrix4>( jinfos_new[jointinfo_id].mat ) = Transform3f(mat).matrix();
                break;
            }
            }
    }

#ifdef PRINT_MEMORY_SIZES
    LOG(INFO) << "JointTransformations" << sizeof(JointTransformations);
    LOG(INFO) << "JointTransformations/2" << sizeof(CustomJointInfo)*jinfos_new.size();
#endif
}

/// TODO: this is only used ONCE (temporal)... why?!?
Matrix_3xN SkeletonSerializer::jacobian(int joint_id_, const Vector3 &position){
    // setup jacobian submatrix
    Mapping mapping = this->getMapping();
    int num = mapping.getNumberOfParameters();
    Matrix_3xN subJ(3, num);
    subJ.setZero();

    // get joint name from constraint's id
    const std::string &name = this->getJointName(joint_id_);
    if(name.empty())
        return subJ;

    // typedef JointTransformations JointTransformations;
    const KinematicChain& adj = this->getKinematicChain();
    const JointTransformations& jinfos_new = this->getJointTransformations();

    /// @note WHY IS THIS DIFFERENT FROM THE joint_id_ parameter?
    const int joint_id = this->getID(name);
    for(int i_column=0; i_column<CHAIN_MAX_LENGTH; i_column++){
        int jointinfo_id = adj[joint_id].data[i_column];
        if(jointinfo_id==-1) break;

        const CustomJointInfo& jinfo = jinfos_new[jointinfo_id];
        switch(jinfo.type){
            case TRANSLATION_AXIS:
            {
                Vector3 loc = Eigen::Map<const Vector3>(jinfos_new[jointinfo_id].axis);
                subJ.col(jinfo.index) = Transform3f( Eigen::Map<const Matrix4>(jinfos_new[jointinfo_id].mat) ) * loc;
                break;
            }
            case ROTATION_AXIS:
            default:
            {
                Vector3 loc = Eigen::Map<const Vector3>(jinfos_new[jointinfo_id].axis);
                Vector3 pos(jinfos_new[jointinfo_id].mat[12],jinfos_new[jointinfo_id].mat[13],jinfos_new[jointinfo_id].mat[14]);
                Vector3 axis = Transform3f(Eigen::Map<const Matrix4>(jinfos_new[jointinfo_id].mat)) * loc - pos;
                subJ.col(jinfo.index) = axis.cross(position - pos);
                break;
            }
        }
    }

    // pc space jacobian
    if(mapping.getPCAEnabled())
        subJ = subJ * mapping.getPCs(mapping.getDof());

    return subJ;
}

void SkeletonSerializer::jacobian(Eigen::Matrix<Scalar, 3, num_thetas> & J, Vector3 s, size_t id){
    /*cout << "s = " << s.transpose() << endl;
    cout << "id = " << id << endl;
    VectorN theta = worker->skeleton->get();
    for (int i = 0; i < theta.size(); ++i) {
        cout << theta(i) << ", ";
    } cout << endl;*/

    const KinematicChain& kinematic_chain = this->getKinematicChain();
    const JointTransformations& joint_transformations = this->getJointTransformations();

    for(int i = 0; i < CHAIN_MAX_LENGTH; i ++){
        int ancestor_id = kinematic_chain[id].data[i];
        if(ancestor_id == -1) break;
        const CustomJointInfo& joint_info = joint_transformations[ancestor_id];

        Vector3 u = Eigen::Map<const Vector3>(joint_info.axis);
        Vector3 p = Vector3(joint_info.mat[12], joint_info.mat[13], joint_info.mat[14]);
        Transform3f T = Transform3f( Eigen::Map<const Matrix4>(joint_info.mat));
        Vector3 v = T * u - p;

        switch(joint_info.type){
        case TRANSLATION_AXIS:{
            J.col(joint_info.index) = v;
            break;
        }
        case ROTATION_AXIS:{
            J.col(ancestor_id) = v.cross(s - p);
            break;
        }
        }
    }
}

inline Vector3 toVec3(float a[3]){ return Vector3(a[0], a[1], a[2]); }

void SkeletonSerializer::reset_hand(){
    this->reset();
    this->translate(toVec3(skeleton_translation));
    this->rotate( Vector3(0,0,1), skeleton_orientation);
}
