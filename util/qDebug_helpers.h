#pragma once
#include <QDebug>
#include <string>
#include "tracker/Types.h"

namespace Eigen{
    inline QDebug operator<< (QDebug d, const BBox2i& box) { return d << "["<< box.min().x() << box.min().y() << box.min().z() << "]"<< "["<< box.max().x() << box.max().y() << box.max().y() << "]"; }
    inline QDebug operator<< (QDebug d, const BBox3& box) { return d << "["<< box.min().x() << box.min().y() << "]"<< "["<< box.max().x() << box.max().y() << "]"; }
    
    // @note these two give problems, replaced with generic matrix
    // inline QDebug operator<< (QDebug d, const Vector2i vec) { return d << "["<< vec.x() << vec.y() << "]"; }
    // inline QDebug operator<< (QDebug d, const Vector3 vec) { return d << "["<< vec.x() << vec.y() << vec.z() << "]"; }
    
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

/// QDebug can accept std::string now!
inline QDebug operator<< (QDebug d, const std::string& str) { return d << QString(str.c_str()); }

inline void qDebugHex(int value){
    qDebug() << QString("%1").arg(value, 8, 16, QLatin1Char('0'));
}

