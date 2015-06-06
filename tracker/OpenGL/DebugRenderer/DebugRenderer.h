#pragma once
#include <map>
#include <memory>
#include "util/singleton.h"
#include "tracker/Types.h"
#include "tracker/OpenGL/ObjectRenderer.h"
#include "CloudRenderer.h"
#include "Segment_renderer.h"

class DebugRenderer : public QObject{
    SINGLETON(DebugRenderer)
    
/// @{
public:
    void add_points(const std::vector<Vector3>& points, Vector3 color);
    void add_points(const std::vector<Vector3>& points, const std::vector<Vector3>& colors);
    void add_segments(const std::vector<pair<Vector3, Vector3>>& segments, Vector3 color);
    void add_segments(const std::vector<pair<Vector3, Vector3>>& segments, const std::vector<Vector3>& colors);
/// @}

/// @{ Functionality to add debug 3D objects in the scene
private:
    int next_free_id=0;
    std::multimap<int,std::shared_ptr<ObjectRenderer>> debug_objects;
public:
    void clear(){ debug_objects.clear(); }
private:
    template <class Derived>
    Derived* add(Derived* object) {
        std::shared_ptr<ObjectRenderer> sptr(object);
        debug_objects.insert( std::make_pair(next_free_id++,sptr) );
        return static_cast<Derived*>(sptr.get());
    }
public:
    void init(){}
    void render(){
        for(auto& entry: debug_objects){
            entry.second->render();
        }
    }
    void set_uniform(const char* name, const Eigen::Matrix4f& value){
        for(auto& entry: debug_objects){
            entry.second->set_uniform(name, value);        
        }
    }
/// @}
};
