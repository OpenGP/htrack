#pragma once
#include <QTimer>
#include <QObject>
#include "util/mylogger.h"
#include "util/tictoc.h"
#include "tracker/ForwardDeclarations.h"
#include "tracker/Sensor/Sensor.h"
#include "tracker/Data/DataStream.h"
#include "tracker/Worker.h"
#include "tracker/Data/SolutionStream.h"
#include "tracker/DataStructure/SkeletonSerializer.h"
#include "tracker/Detection/QianDetection.h"
#include "tracker/Data/TextureColor8UC3.h"
#include "tracker/Data/TextureDepth16UC1.h"
#include "tracker/TwSettings.h"

class Tracker : public QTimer{
public:
    enum Mode{LIVE,RECORD} mode = LIVE;
    Sensor* sensor;
    DataStream* datastream;
    SolutionStream* solutions;
    Worker*const worker=NULL;


///@{ stored only to show results on screen
public:
    float current_fps = 0;
    bool track_failed = true;
    bool init_enable = true;
    bool aicp_enabled = true;
///@}

public:
    Tracker(Worker*worker, double FPS) : worker(worker){
        setSingleShot(false);
        setInterval((1.0/FPS)*1000.0);
        tw_settings->tw_add_ro(current_fps,"FPS","group=Tracker");
        tw_settings->tw_add(init_enable, "Detect ON?","group=Tracker");
        tw_settings->tw_add(aicp_enabled,"ArtICP ON?","group=Tracker");
        tw_settings->tw_add_ro(track_failed,"Tracking Lost?","group=Tracker");
    }
    void toggle_recording(bool on){
        mode = RECORD;
        solutions->reserve(30/*FPS*/ *60/*SEC*/ *3/*MIN*/ );
        on ? start() : stop();
#if 0
        /// TODO: broken Qt UI scrubbing
        if(!on){
            if(settings->track_while_recording)
                solutions->setValid(); ///< enable scrubbing
        }
#endif
    }
    void toggle_tracking(bool on){
        mode = LIVE;
        if(on)
        {
            sensor->start();
            if( sensor->spin_wait_for_data(5 /*seconds*/) == false ){
                LOG(INFO) << "!!! Please replug the (friggin) kinect.";
                //QApplication::exit(0);    
            }
            start(); ///< timerEvent
        }
        else
        {
            stop(); ///< timerEvent
            sensor->stop();
        }
    }
private:
    void timerEvent(QTimerEvent*){ process_track(); }
    
    void process_track(){
        // TIMED_SCOPE(total,"TOTAL");
		tic(total);
        static int frame_offset=0;

        // TICTOC_BLOCK(fetch_time,"Kinect Fetch")
        {
            bool success = sensor->fetch_streams(worker->current_frame);
            assert(success);          
        }

        // TICTOC_BLOCK(init_time,"Data tex upload GPU")
        {
            ///--- Save the data
            if(mode==RECORD)
                frame_offset = datastream->add_frame(worker->current_frame.color.data, worker->current_frame.depth.data);            
            else
                frame_offset++;
            worker->current_frame.id = frame_offset;
            
            ///--- Upload the data
            worker->sensor_color_texture->load(worker->current_frame.color.data, worker->current_frame.id);
            worker->sensor_depth_texture->load(worker->current_frame.depth.data, worker->current_frame.id);
        }

        // TICTOC_BLOCK(track_time,"Track Time")
        {
            // If ICP doesn't track, just assume it has failed
            track_failed = aicp_enabled?worker->track_till_convergence(worker->current_frame):true;
            glFinish();
        }
        if(init_enable && track_failed ){            
            static QianDetection detection(worker);
            if( detection.can_reinitialize() )
                detection.reinitialize();
        }

        // TIMED_BLOCK(track_time,"Rendering")
        {
            worker->updateGL();
            glFinish();
        }

        ///--- Tracker execution time
        current_fps = 1000.0/toc(total);
        // printf("TOT: %2.1f\n", toc(total)); fflush(stdout);
        
        ///--- Save the solution
        if(mode==RECORD && (aicp_enabled||init_enable) ){
            solutions->resize(datastream->size());
            solutions->set(frame_offset, worker->skeleton->getCurrentParameters());
        }
    }
};
