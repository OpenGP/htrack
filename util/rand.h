#pragma once
#include "tracker/Types.h"

//== NAMESPACE ================================================================
namespace my {
//=============================================================================

/// see matlab randn
static inline Scalar randn( const double mean, const double stddev ){
    double u1 = ( static_cast<double> ( std::rand() ) + 1 ) / ( (double)RAND_MAX + 1 );
    double u2 = ( static_cast<double> ( std::rand() ) + 1 ) / ( (double)RAND_MAX + 1 );
    assert( -2 * std::log( u1 ) >= 0 );
    double t1 = std::sqrt( -2 * std::log( u1 ) ) * cos( 2 * M_PI * u2 );
    return mean + stddev * t1;
}
static inline Scalar rand(){
    /// @todo check this one actually gives [0..1]
    return ( static_cast<double> ( std::rand() ) ) / ( (double)RAND_MAX );
}

//=============================================================================
} // namespace my
//=============================================================================
