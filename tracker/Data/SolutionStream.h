#pragma once
#include "tracker/Types.h"
#include <vector>
#include <QString>
#include <fstream>
#include <stdio.h>
#include "tracker/Data/DataStream.h"

///--- This is only valid when we record a stream
class SolutionStream{
    std::vector< Thetas > frames;
    bool _valid = false;
    
public:
    bool isValid(int fid = 0){
        return _valid; // || ( (fid <= _valid_id) && (fid >= 0) );
    }
    void setValid(){
        _valid = true;
    }
    void invalidate(int fid){ 
        _valid = false;
    } 
    
    void reserve(int size){
        frames.reserve(size);
    }

    void resize(int num_frames){
        // Frame zero_frame(num_thetas,0);
        frames.resize(num_frames);
        // frames.fill(0);
    }
    
    void set(int frame_id, const std::vector<Scalar>& theta ){
        Eigen::Map<const Thetas> _theta(theta.data());
        frames[frame_id] = _theta;
    }
    
    std::vector<Scalar> get(int frame_id){
        if(!isValid(frame_id))
            return std::vector<Scalar>();
        std::vector<Scalar> retval(num_thetas);
        Eigen::Map<Thetas> _retval(retval.data());
        _retval = frames[frame_id];
        return retval;
    }
        
    void save(QString filename){
        std::ofstream out(filename.toStdString());
        out << "version #1" << endl;
        out << frames.size() << " " << num_thetas << endl;

        for (int i = 0; i < frames.size(); ++i)
            out << frames[i].transpose() << endl;            

#if 0
        ///--- Now dump datastream filename
        out << "################### datastream ###################" << endl;
        out << datastream->filename().toStdString() << endl;
#endif

#if 0
        ///--- Now dump settings->ini file into it
        out << "################### settings ###################" << endl;
        std::ifstream in(settings->fileName().toStdString());
        for (std::string line; std::getline(in, line);)
            out << line << std::endl;
        in.close();
#endif

        out.close();
    }
    
    void load(QString filename){
        TICTOC_SCOPE(timer,"SolutionStream::load()");
        
        std::string line;
        std::ifstream in(filename.toStdString());
        if(!in.is_open()){
            LOG(INFO) << "!!!WARNING: could not load file " << filename.toStdString();
            exit(0);
        }
                        
        ///--- Read header
        std::getline(in, line);
        LOG(INFO) << "loading solution file " << line;
        
        ///--- Read size
        std::getline(in, line);
        stringstream str(line);
        int num_frames;
        int num_thetas;
        str>>num_frames;
        str>>num_thetas;
        LOG(INFO) << "num frames: " << num_frames;
        LOG(INFO) << "num thetas: " << num_thetas;
        
        ///--- Allocate
        frames.resize(num_frames);
        
        ///--- Read in the matrix
        int row=0;
        for (std::string line; std::getline(in, line) && row<num_frames; row++) {
            stringstream str(line);
            for (int col = 0; col < num_thetas; ++col) {
#if __unix__
                // M: my std::stof produces ints instead of floats.. streaming
                //    directly into a float variable instead works on linux,
                //    not sure if portable/safe though.
                Scalar val;
                str >> val;
                frames[row](col) = val;
#else
                std::string elem;
                str >> elem;
                frames[row](col) = std::stof(elem);
#endif
            }
        }
        
        // std::ofstream("dump.txt")<<frames;
        in.close();
        
        _valid = true;
    }
};
