#pragma once
#include <string>

class Cylinders;
class SkeletonSerializer;
class Worker;

class Calibration{
private:
    SkeletonSerializer*const skeleton;
    Cylinders*const cylinders;

public:
    Calibration(Worker* worker);
    void save(const std::string &filename);
    /// TODO: why we must call "glarea->reload_model"?
    void load(const std::string &filename);
    /// TODO: why we must call "glarea->reload_model"?
    void autoload();
    /// Helper for interactive template adjustment
    static void update_radius_helper(Cylinders* cylinders, SkeletonSerializer* skeleton);
};
