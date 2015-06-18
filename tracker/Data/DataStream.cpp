#include "DataStream.h"
#include <QFileDialog>
#include <QByteArray>
#include <QFile>

#include "util/qt2eigen.h"
#include "util/mylogger.h"
#include <algorithm>
#include <fstream>

DataStream::DataStream(Camera *camera) : _camera(camera){
    assert( camera != NULL);
}

DataStream::~DataStream(){
    for(uint i=0; i<frames.size(); i++)
        delete frames.at(i); 
}

int DataStream::add_frame(const void* color_buffer, const void* depth_buffer){
    // TIMED_FUNC(time);
    // qDebug() << "DataStream::add_frame()";
    /// Attach a new frame
    frames.push_back( new DataFrame(frames.size()) );
    DataFrame& frame = *frames.back();
    
    /// Clone the data
    if(color_buffer) frame.color = cv::Mat(height(), width(), CV_8UC3, (void*) color_buffer).clone();
    if(depth_buffer) frame.depth = cv::Mat(height(), width(), CV_16UC1, (void*) depth_buffer).clone();
    if(!color_buffer) mDebug() << "warning: null color buffer?";
    if(!depth_buffer) mDebug() << "warning: null depth buffer?";
    
//#define TOMPSON_COLOR_IMAGE_FIX
#ifdef TOMPSON_COLOR_IMAGE_FIX
    cv::cvtColor(frame.color, frame.color, CV_BGR2RGB);
#endif
    
    /// Signal system to update GUI
    return (frames.size()-1);
}

DataFrame&DataStream::get_frame(int id){
    // CHECK_BOUNDS(id, frames.size()); 
    if(id < 0){
        mWarning() << "frame_id out of bounds";
        return *(frames.at(0));
    }
    if(id >= frames.size()){
        mWarning() << "frame_id out of bounds";        
        return *(frames.at(frames.size()-1));
    }
    return *(frames.at(id)); 
}

/// (start/end are included in the result)
void DataStream::crop(int start, int end){
    CHECK_BOUNDS(start, 0, frames.size());
    CHECK_BOUNDS(end, 0, frames.size());
    CHECK(start<end);
    
    /// mark frame id as invalid in the range
    for(int i=0; i<frames.size(); i++)
        if(i<start || i>end)
            frames.at(i)->id = -1;
  
    /// now let std take over
    auto lambda = [](const DataFrame* f) { return (f->id==-1); };
    frames.erase(std::remove_if(frames.begin(), frames.end(), lambda), frames.end());
}

QString DataStream::get_prefix()
{
    return get_basename().split("_")[0];
}

QString DataStream::get_basename()
{
    return QFileInfo(_filename).baseName();
}

void DataStream::save(QString path){
    if(frames.size()==0){
        mWarning() << "WARNING: not saving an empty stream";
        return;
    }
    
    QFile file(path);
    file.open(QIODevice::WriteOnly);
    QDataStream out(&file);
    
    /// Header (#frames, image size)
    int num_frames = frames.size();
    int color_frame_size = frames[0]->color.total()*3; ///< was 3 because RGB=3xUCHAR?
    int depth_frame_size = frames[0]->depth.total()*2; ///< was 2 because USHORT=2xUCHAR?
    out << num_frames;
    out << color_frame_size;
    out << depth_frame_size;
    
    /// Data
    foreach(const DataFrame* frame, frames){
        out << QByteArray::fromRawData((const char*) frame->color.data, color_frame_size);
        out << QByteArray::fromRawData((const char*) frame->depth.data, depth_frame_size);
        // break; /// @REMOVE ME
    }
    file.close();

    _filename = path;
}

void DataStream::load(QString path){
    if(path.isEmpty()) return;
    _filename = path; ///< used below
    
    QFile file(path);
    if(!file.exists()){
        LOG(INFO) << "Creating stream at: " << file.fileName().toStdString();
        return;
    } else {
        LOG(INFO) << "Loading stream from: " << file.fileName().toStdString();                    
    }
    
    ///--- Setup the camera properly
    if( is_tompson() ){
        LOG(INFO) << "Loading 'tompson' camera";
        *_camera = Camera(Tompson);
    } else if(is_tang()){
        LOG(INFO) << "Loading 'tang' camera";
        *_camera = Camera(Tang);
    } else if(is_chen()){
        LOG(INFO) << "Loading 'chen' camera";
        *_camera = Camera(Chen);
    } else if(is_colorglove()){
        LOG(INFO) << "Loading 'VGA' camera";
        *_camera = Camera(VGA);
    } else if(is_sridhar()){
        LOG(INFO) << "Loading 'Intel' camera";
        *_camera = Camera(Intel);
    } else {
        LOG(INFO) << "Loading 'main.cpp' camera";
    }
    
    if(path.endsWith(".dat")) {
        // frames.clear();        
        
        file.open(QIODevice::ReadOnly);
        QDataStream in(&file);
        //in.setByteOrder(QDataStream::LittleEndian);
        
        /// Header
        int num_frames;
        int color_frame_size;
        int depth_frame_size;
        in >> num_frames;
        in >> color_frame_size;
        in >> depth_frame_size;
        
        /// Default bounding box
        // BBox2i bbox = Convert::tr( settings->value("default_bbox").toRect() );
        // std::cout << bbox.min() << "\n" << bbox.max() << std::endl;
        
        qDebug("Loading %d frames...", num_frames);
        for(int i=0; i<num_frames; i++){
            QByteArray color; in >> color;
            QByteArray depth; in >> depth;
            
            add_frame(color.data(), depth.data());
        }
        
        file.close();
    } else {
        std::ifstream in(path.toStdString().c_str(), ios::in | ios::binary);
        /// Header
        int num_frames;
        int color_frame_size;
        int depth_frame_size;
        in.read((char*)&num_frames, sizeof(int));
        in.read((char*)&color_frame_size, sizeof(int));
        in.read((char*)&depth_frame_size, sizeof(int));
        cout << "num_frames: " << num_frames << endl;
        cout << "color_frame_size: " << color_frame_size << endl;
        cout << "depth_frame_size: " << depth_frame_size << endl;
        for(int i=0; i<num_frames; i++){
            unsigned char *color = new unsigned char[color_frame_size];
            unsigned char *depth = new unsigned char[depth_frame_size];
            in.read((char*)color, color_frame_size);
            in.read((char*)depth, depth_frame_size);
            add_frame(color, depth);
        }
        in.close();
    }
}
