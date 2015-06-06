#pragma once
#include <QList>

#include "tracker/ForwardDeclarations.h"
#include "tracker/Types.h"
#include "Camera.h"
#include "DataFrame.h"
#include <QString>

class DataStream{       
private:
    /// @note frames.at(idx) is a constant time operation!! 
    QList<DataFrame*> frames;
    /// where was stream loaded/saved from/to?
    QString _filename;
    
/// @{ camera management
private:
    Camera* _camera;
public:
    Camera& camera(){ return *_camera; }
/// @}
    
/// @{ frame count/size
public:
    const QString& filename(){ return _filename; }
    int width() const { return _camera->width(); }
    int height() const { return _camera->height(); }
    int num_frames() const{ return frames.size(); }
    int size() const{ return frames.size(); }
/// @}

public:
    int add_frame(const void* color_buffer, const void* depth_buffer);
    DataFrame& get_frame(int id);
    void crop(int start, int end);
    QString get_basename();
    QString get_prefix();
    bool is_synthetic(){ return get_prefix().startsWith("synth", Qt::CaseInsensitive); }
    bool is_tompson(){ return get_prefix().startsWith("tompson", Qt::CaseInsensitive); }
    bool is_tang(){ return get_prefix().startsWith("tang", Qt::CaseInsensitive); }
    bool is_chen(){ return get_prefix().startsWith("chen", Qt::CaseInsensitive); }
    bool is_colorglove(){ return get_prefix().startsWith("colorglove", Qt::CaseInsensitive); }
    bool is_legacy_maschroe(){ return get_basename().startsWith("maschroe_sequence_", Qt::CaseInsensitive); }
    bool is_melax(){ return _filename.endsWith(".bin"); }
    bool is_sridhar(){ return get_prefix().startsWith("sridhar", Qt::CaseInsensitive); }
public:
    DataStream(Camera* camera);
    ~DataStream();
public:
    void save(QString path);
    void load(QString path);
};



