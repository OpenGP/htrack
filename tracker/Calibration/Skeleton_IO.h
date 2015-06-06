#pragma once
class Skeleton;
#include <string>

class Skeleton_IO{
    Skeleton *_skeleton;
public:
    Skeleton_IO(Skeleton *skeleton) : _skeleton(skeleton){}
    Skeleton_IO(const std::string &filename) { read(filename); }
    void read(const std::string &filename);
    void write(const std::string &filename);
    Skeleton* skeleton(){ return _skeleton; }
};

