#include "JointLimits.h"
#include "tracker/DataStructure/SkeletonSerializer.h"

void energy::JointLimits::init(SkeletonSerializer *skeleton){
    this->skeleton = skeleton;
}

void energy::JointLimits::track(LinearSystem& sys, const std::vector<Scalar>& theta_0)
{
    if(!jointlimits_enable) return;
    
    //static Matrix_MxN LHS = Matrix_MxN::Identity(sys.lhs.rows(), sys.lhs.cols());
    //static VectorN rhs = VectorN::Ones(sys.lhs.cols(), 1);
    Eigen::Matrix<Scalar, num_thetas, num_thetas> LHS = Eigen::Matrix<Scalar, num_thetas, num_thetas>::Identity(sys.lhs.rows(), sys.lhs.cols());
    Eigen::Matrix<Scalar, num_thetas, 1>  rhs = Eigen::Matrix<Scalar, num_thetas, 1>::Ones( sys.lhs.rows(),1 );
            
    Mapping mapping = skeleton->getMapping();
    
    bool show_constraints = jointlimits_show_constraints;
    
    // LOG(INFO) << "jointlimits" << rhs.rows() << rhs.cols() << sys.rhs.cols();
    for(size_t i = 0; i < theta_0.size(); ++i)
    {
        float qmin = mapping.getMin(i);
        float qmax = mapping.getMax(i);
        if(theta_0[i]>qmax)
        {
            rhs(i)=(qmax-theta_0[i])-std::numeric_limits<Scalar>::epsilon();
            if(show_constraints) printf("jointlimits[%zu] -> %f\n",i, rhs(i));
            LHS(i,i) = 1;
        }
        else if(theta_0[i]<qmin)
        {
            rhs(i)=(qmin-theta_0[i])+std::numeric_limits<Scalar>::epsilon();
            LHS(i,i) = 1;
        }
        else
        {
            LHS(i,i) = 0;
            rhs(i) = 0;
        }
    }   
    
    ///--- Add constraints to the solve
    Scalar omega = jointlimits_weight;
    sys.lhs.noalias() += omega * LHS.transpose() * LHS;
    sys.rhs.noalias() += omega * LHS.transpose() * rhs;
    
    ///--- Check
    if(Energy::safety_check)
        Energy::has_nan(sys);
}
