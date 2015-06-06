#pragma once
#include "tracker/ForwardDeclarations.h"
#include "Energy.h"
#include "tracker/Types.h"
#include <vector>

namespace energy{

class Collision : public Energy{
public:
    struct Settings{
        bool collision_enable = true;
        float collision_weight = 10e3;
        float collision_fraction = .1f;
        bool collision_new = false; ///< TODO(anastasia), still buggy!
    } _settings;
    Settings*const settings=&_settings;

public:
    struct Point{
        Vector3 p;
        size_t id;
        Point(Vector3 p, size_t id){
            this->p = p;
            this->id = id;
        }
    };


private:
    SkeletonSerializer* skeleton = NULL;
    Cylinders* cylinders = NULL;
    Worker* worker = NULL;
    std::vector<Point> s;
    std::vector<Point> t;
    std::vector<Vector3> n;

public:
    void init(Worker* worker);
    void track(LinearSystem& system);

private:
    void collision_compute_update(LinearSystem &system, Scalar omega);
    Scalar create_collision_constraints();
};

} ///< energy::
