#pragma once
#include <Eigen/Dense>
#include "MLogger.h"

namespace Eigen{
    // @todo bounding boxes
    // inline MLogger operator<< (MLogger d, const BBox2i& box) { return d << "["<< box.min().x() << box.min().y() << box.min().z() << "]"<< "["<< box.max().x() << box.max().y() << box.max().y() << "]"; }
    // inline MLogger operator<< (MLogger d, const BBox3& box) { return d << "["<< box.min().x() << box.min().y() << "]"<< "["<< box.max().x() << box.max().y() << "]"; }
    
    // @note these two give problems, replaced with generic matrix
    // inline QDebug operator<< (QDebug d, const Vector2i vec) { return d << "["<< vec.x() << vec.y() << "]"; }
    // inline QDebug operator<< (QDebug d, const Vector3 vec) { return d << "["<< vec.x() << vec.y() << vec.z() << "]"; }
    
    /// Output of generic Matrix
    template <class Derived>
    inline QDebug operator<< (QDebug d, const MatrixBase<Derived>& m){ 
        for(int i=0; i<m.rows(); i++){
            for(int j=0; j<m.cols(); j++)
                d << m(i,j) << " ";
            d << "\n";
        }
        return d;
    }
}
