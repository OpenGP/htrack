#include "Damping.h"
#include "tracker/DataStructure/SkeletonSerializer.h"
#include "tracker/TwSettings.h"

namespace energy{

void Damping::init(SkeletonSerializer *skeleton){
    this->skeleton = skeleton;
    tw_settings->tw_add(settings->intertial_damping_translation,"translations","group=Damping");
    tw_settings->tw_add(settings->intertial_damping,"rotations","group=Damping");
}

void Damping::track(LinearSystem &system){
    Mapping mapping = skeleton->getMapping();
    const int num = mapping.getDimension();
    Matrix_MxN D = Matrix_MxN::Identity(system.lhs.rows(), system.lhs.cols());
    VectorN d = VectorN::Ones( system.lhs.rows() );
    //Eigen::Matrix<Scalar, num_thetas, num_thetas> D = Eigen::Matrix<Scalar, num_thetas, num_thetas>::Identity(system.lhs.rows(), system.lhs.cols());
    //Eigen::Matrix<Scalar, num_thetas, 1>  d = Eigen::Matrix<Scalar, num_thetas, 1>::Ones( system.lhs.rows(),1 );
    
    for(int i = 0; i < num; ++i){
        if(mapping.getJointInfo(i).type == TRANSLATION_AXIS)
            d(i) = settings->intertial_damping_translation;
        else
            d(i) = settings->intertial_damping;
    }
    D.diagonal() = d;
    system.lhs = system.lhs + D;
    
    ///--- Check
    if(Energy::safety_check)
        Energy::has_nan(system);
}

} // energy::
