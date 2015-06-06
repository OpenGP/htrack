#include "Energy.h"
#include "util/mylogger.h"
namespace energy{

static bool debug_print_delta_theta = false;

bool Energy::has_nan(LinearSystem& system){
    for(int i=0; i<system.lhs.rows(); i++){
        for(int j=0; j<system.lhs.cols(); j++){
            if( isnan( system.lhs(i,j)) ){
                LOG(INFO) << "--------------------------------------------------------";
                LOG(INFO) << "!!!WARNING: NaN DETECTED in the LHS!!! (skipping update)";
                LOG(INFO) << "--------------------------------------------------------";
                return true;
            }
            if( isinf( system.lhs(i,j)) ){
                LOG(INFO) << "--------------------------------------------------------";
                LOG(INFO) << "!!!WARNING: inf DETECTED in the LHS!!! (skipping update)";
                LOG(INFO) << "--------------------------------------------------------";
                return true;
            }
        }
        if( isnan( system.rhs(i)) ){
            LOG(INFO) << "--------------------------------------------------------";
            LOG(INFO) << "!!!WARNING: NaN DETECTED in the RHS!!! (skipping update)";
            LOG(INFO) << "--------------------------------------------------------";
            return true;
        }
        if( isinf( system.rhs(i)) ){
            LOG(INFO) << "--------------------------------------------------------";
            LOG(INFO) << "!!!WARNING: inf DETECTED in the RHS!!! (skipping update)";
            LOG(INFO) << "--------------------------------------------------------";
            return true;
        }
    }
    return false;
}

void Energy::rigid_only(LinearSystem &system){
    int start = 6; ///< fully rigid
    // int start = 8; ///< wrist DOF enabled
    for (int c=start; c < num_thetas; ++c)
        system.lhs.col(c).setZero();
    for (int r=start; r < num_thetas; ++r)
    {
        system.lhs.row(r).setZero();
        system.rhs.row(r).setZero();
    }
    // if(has_nan(system)) cout << "NAN " << __LINE__ << endl;
}

VectorN Energy::solve(LinearSystem &system){
    ///--- Solve for update dt = (J^T * J + D)^-1 * J^T * r
    VectorN solution = system.lhs.colPivHouseholderQr().solve(system.rhs);

    ///--- Check for NaN
    for(int i=0; i<solution.size(); i++){
        if( isnan( solution[i]) ){
            LOG(INFO) << "-------------------------------------------------------------";
            LOG(INFO) << "!!!WARNING: NaN DETECTED in the solution!!! (skipping update)";
            LOG(INFO) << "-------------------------------------------------------------";
            return VectorN::Zero(num_thetas,1);
        }
    }

    ///--- Only take portion of solution for hand
    VectorN delta_thetas = solution.head(num_thetas);

    ///--- Debug
    if(debug_print_delta_theta)
        LOG(INFO) << "delta_thetas: " << delta_thetas.transpose();

    return delta_thetas;
}

} /// energy::
