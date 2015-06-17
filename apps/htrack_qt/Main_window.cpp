#include "Main_window.h"

#include <QFileDialog>
#include <QThread>
#include "OpenGL_viewer.h"
#include "tracker/OpenGL/DebugRenderer/DebugRenderer.h"
#include "cudax/Timer.h"
#include "util/opencv_wrapper.h"
#include "util/mylogger.h"

#include "tracker/Worker.h"
#include "tracker/Tracker.h"
#include "tracker/Data/SolutionStream.h"
#include "tracker/Data/DataStream.h"
#include "tracker/Calibration/Calibration.h"
#include "tracker/HandFinder/HandFinder.h"
#include "tracker/DataStructure/SkeletonSerializer.h"

Main_window::Main_window(Worker* worker, DataStream* stream, SolutionStream* solutions, OpenGL_viewer* qglviewer):
    worker(worker), datastream(stream), solutions(solutions), qglviewer(qglviewer){
    /// Delete class when window closes
    this->setAttribute(Qt::WA_DeleteOnClose);


    QWidget* widget = new QWidget(this);
    QVBoxLayout* v_layout = new QVBoxLayout;
    //v_layout->setAlignment(Qt::AlignCenter);
    widget->setLayout(v_layout);
    this->setCentralWidget(widget);

    /// Part 2: opengl
    {
        // qglviewer->setFixedSize(870, 870);
        v_layout->addWidget(qglviewer);
        // qglviewer->show();
    }

    /// Part 3: scrollbar
    {
        QHBoxLayout* h_layout = new QHBoxLayout;
        slider = new QSlider(Qt::Horizontal);
		slider->setMaximumHeight(24);
        slider_label = new QLabel("0/0");
		slider_label->setMaximumHeight(24);
        toggle_live_ = new QPushButton("LIVE");
		toggle_live_->setMaximumHeight(24);
        toggle_live_->setCheckable(true);
        toggle_live_->setChecked(false);
        h_layout->addWidget(toggle_live_);
        toggle_record_ = new QPushButton("REC");
		toggle_record_->setMaximumHeight(24);
        toggle_record_->setCheckable(true);
        toggle_record_->setChecked(false);
        h_layout->addWidget(toggle_record_);
        h_layout->addWidget(slider);
        h_layout->addWidget(slider_label);
        v_layout->addLayout(h_layout);
    }

    ///--- Events
    connect(slider, SIGNAL(sliderMoved(int)), this, SLOT(display_frame(int)));

    /// Menu
    {
        //--- File menu
        QMenu* file = menuBar()->addMenu("File");
        {
            QAction* save = file->addAction( "Save" );
            save->setShortcut( QKeySequence(Qt::CTRL + Qt::Key_S) );
            connect(save, SIGNAL(triggered()), this, SLOT(save_slot()));
        }
        {
            QAction* load = file->addAction( "Open" );
            load->setShortcut( QKeySequence(Qt::CTRL + Qt::Key_O) );
            connect(load, SIGNAL(triggered()), this, SLOT(load_slot()));
        }
        {
            connect(file->addAction("Save frame"), SIGNAL(triggered()), this, SLOT(save_frame_slot()));
        }
        file->addSeparator();
        {
            QAction* action = file->addAction( "Save tracking" );
            connect(action, SIGNAL(triggered()), this, SLOT(save_tracking_pressed()));
        }
        {
            QAction* action = file->addAction( "Load tracking" );
            connect(action, SIGNAL(triggered()), this, SLOT(load_tracking_pressed()));
        }
        file->addSeparator();        
        {
            QAction* action = file->addAction( "Save results" );
            connect(action, SIGNAL(triggered()), this, SLOT(save_results_pressed()));
        }
        {
            QAction* action = file->addAction( "Save results (one frame)" );
            connect(action, SIGNAL(triggered()), this, SLOT(save_results_single_pressed()));
        }
        {
            QAction* action = file->addAction( "Save joint positions (seq.)" );
            connect(action, SIGNAL(triggered()), this, SLOT(save_joint_position_list()));
        }
                
        //--- Track menu
        QMenu* track = menuBar()->addMenu("Track");
        {
            // worker->initialize_offset(frame);
            QAction* action = track->addAction( "Offset initialization" );
            action->setShortcut( Qt::Key_I );
            connect(action, SIGNAL(triggered()), this, SLOT(menu_initialize_offset()));
        }
        {
            QAction* action = track->addAction("Re-Init from previous frame");
            action->setShortcut( Qt::Key_F2 );
            connect(action, SIGNAL(triggered()), this, SLOT(reinit_from_previous_frame()));
        }
        {
            QAction* action = track->addAction( "Optimize current (1x)" );
#ifdef __unix__
            action->setShortcut( Qt::Key_T );
#else
            action->setShortcut( Qt::Key_F1 );            
#endif
            connect(action, SIGNAL(triggered()), this, SLOT(menu_track_pressed()));
        }
        {
            QAction* action = track->addAction( "Optimize current (convergence)" );
            // action->setShortcut( Qt::Key_F2 );
            connect(action, SIGNAL(triggered()), this, SLOT(menu_track_till_convergence_pressed()));
        }
        {
            A_track_whole = track->addAction( "Re-Optimize whole sequence" );
            A_track_whole->setShortcut( Qt::Key_F3 );
            A_track_whole->setCheckable(true);
            connect(A_track_whole, SIGNAL(toggled(bool)), this, SLOT(track_whole_sequence(bool)));
        }
        {
            QAction* action = track->addAction( "Set current solution" );
            // action->setShortcut( Qt::Key_S );
            connect(action, SIGNAL(triggered()), this, SLOT(menu_set_current_solution()));
        }
        {
            A_replay = track->addAction( "Replay sequence" );
            A_replay->setShortcut( Qt::Key_Space );
            A_replay->setCheckable(true);
            connect(A_replay, SIGNAL(toggled(bool)), this, SLOT(replay_sequence(bool)));
        }
        {
            QAction* action = track->addAction( "Next frame" );
            action->setShortcut( Qt::Key_Right );
            connect(action, SIGNAL(triggered()), this, SLOT(display_next_frame()));
        }
        {
            QAction* action = track->addAction( "Prevous frame" );
            action->setShortcut( Qt::Key_Left );
            connect(action, SIGNAL(triggered()), this, SLOT(display_previous_frame()));
        }
        
        QMenu* extra = menuBar()->addMenu("Extra");
        {        
            connect(extra->addAction("Save Calibration"), SIGNAL(triggered()), this, SLOT(save_calibration_slot()));
            connect(extra->addAction("Load Calibration"), SIGNAL(triggered()), this, SLOT(load_calibration_slot()));
            extra->addSeparator();            
            connect(extra->addAction("Reload Model"), SIGNAL(triggered()), qglviewer, SLOT(reload_model()));
        }
    }

    /// I have data on load
    if (stream->size()>1) {
        qDebug() << "#available frames" << stream->num_frames()-1;
        slider->setRange(0,stream->num_frames()-1);
        display_frame(0);
    }
}

void Main_window::requested_track(int fid)
{
    // LOG(INFO) << "Main_window::requested_track";
    //solutions->invalidate(fid); ///< we are building it
    DataFrame& frame = datastream->get_frame(fid);
    worker->track(frame,false,false);
    solutions->set(fid, worker->skeleton->getCurrentParameters());
    qglviewer->updateGL();
}

void Main_window::reinit_from_previous_frame(){
    int fid = slider->value();
    if(fid==0) return;
    worker->skeleton->set( solutions->get(fid-1) );   
}

void Main_window::requested_track_till_convergence(int fid){
    // solutions->invalidate(fid); ///< we are building it
    DataFrame& frame = datastream->get_frame(fid);
    worker->track_till_convergence(frame);
    solutions->set(fid, worker->skeleton->getCurrentParameters());
    qglviewer->updateGL();
}

void Main_window::track_whole_sequence(bool checked){
    ///--- Don't restart if disabling
    if(checked==false) return;
    
    int start_at = slider->value();
    
    // LOG(INFO) << "!!!Warning only doing first 10";
    solutions->invalidate(start_at); ///< we are building it
    solutions->resize(datastream->size());
    for (int fid = start_at; fid < datastream->num_frames(); fid++){
        DataFrame& frame = datastream->get_frame(fid);
        display_frame(fid);

        ///--- Track
        worker->track_till_convergence(frame);
        ///--- Save
        solutions->set(fid, worker->skeleton->getCurrentParameters());
        
        qglviewer->updateGL();
        QApplication::processEvents();
        
        ///--- Allow user to interrupt
        if(A_track_whole->isChecked()==false)
            break;
    }
    
    ///--- If we did all this work, we might as well just save it...
    solutions->save("tracks/last_track_whole_sequence.track");
    solutions->setValid(); ///< we are done building
    A_track_whole->setChecked(false);
}

void Main_window::set_current_solution(int fid){
    solutions->set(fid, worker->skeleton->getCurrentParameters());
    qglviewer->updateGL();
}

void Main_window::replay_sequence(bool checked){
    if(checked==false) return;
    float interval = 1000.0f / worker->camera->FPS();
    
    cudax::Timer t;
    for (int fid = 0; fid < datastream->num_frames(); fid++){
        t.restart("");

        DataFrame& frame = datastream->get_frame(fid);
        display_frame(fid);
        QApplication::processEvents();

        ///--- Allow user to interrupt
        if(A_replay->isChecked()==false)
            break;

        // limit replay fps
        float dt = t.elapsed();
        float ms = interval- dt - 8.0f; // which 8ms?!
        if(ms>0) QThread::usleep(ms*1000);
    }
}

void Main_window::display_next_frame(){
    int fid = slider->value();
    display_frame(++fid);
}

void Main_window::display_previous_frame(){
    int fid = slider->value();
    display_frame(--fid);
}

void Main_window::display_latest_frame(){
    int fid = datastream->size()-1;
    
    display_frame(fid);
}

void Main_window::crop(int from, int to)
{
    datastream->crop(from, to);
    this->display_latest_frame();
}

void Main_window::save_tracking_pressed()
{
    QString filename = QFileDialog::getSaveFileName(QApplication::activeWindow(), tr("Save tracking as..."), QDir::currentPath()+"/tracks");
    solutions->save(filename);
}

void Main_window::load_tracking_pressed()
{
    QString filename = QFileDialog::getOpenFileName(QApplication::activeWindow(), tr("Load tracking"), QDir::currentPath()+"/tracks");
    solutions->load(filename);
}

void Main_window::save_results_pressed()
{
    ///--- Pick a folder
    QDir().mkdir("results"); ///< just returns false if exists
    QString fname = QFileDialog::getExistingDirectory(QApplication::activeWindow(), 
                                                      "Save Image Sequence in folder...", 
                                                      QDir::currentPath()+"/results" 
                                                      /*, QFileDialog::ShowDirsOnly*/);
    if(fname.isNull())
        return;
    
    ///--- Save images
    qglviewer->transparent_background = true;
    for (int fid = 0; fid < datastream->num_frames(); fid++){
        worker->skeleton->set( solutions->get(fid) );
        display_frame(fid);
        qglviewer->updateGL();
        QApplication::processEvents();
        glFinish();
        ///--- Time to save
        QString filename = fname + QString("/%1.png").arg(fid, /*pad to 10k*/ 6, /*base*/ 10, /*fillchar*/ QLatin1Char('0'));
#define SAVE_RESULTS_EXPORT
#ifdef SAVE_RESULTS_EXPORT
        QImage img = qglviewer->grabFrameBuffer(/*withAlpha=*/false);
#else
        QImage img = glarea->grabFrameBuffer(/*withAlpha=*/true);
#endif
        img.save(filename);
    }
    qglviewer->transparent_background = false;
}

void Main_window::save_results_single_pressed()
{
    QString fname = QFileDialog::getSaveFileName(QApplication::activeWindow(), 
                                                 "Save Image in file...", 
                                                 QDir::currentPath()+"/results");
    if(fname.isEmpty()) return;
    
    qglviewer->transparent_background = true;
    {
        int fid = slider->value();
        worker->skeleton->set( solutions->get(fid) );
        display_frame(fid);
        qglviewer->updateGL();
        QApplication::processEvents();
        glFinish();
        ///--- Time to save  
#define SAVE_RESULTS_SINGLE_EXPORT
#ifdef SAVE_RESULTS_SINGLE_EXPORT
        QImage img = qglviewer->grabFrameBuffer(/*withAlpha=*/false);
#else
        QImage img = qglviewer->grabFrameBuffer(/*withAlpha=*/true);
#endif
        img.save(fname);
    }
    qglviewer->transparent_background = false;
}

void Main_window::menu_initialize_offset()
{
    worker->handfinder->binary_classification(worker->current_frame);
    worker->trivial_detector->exec(worker->current_frame, worker->handfinder->sensor_silhouette);
    qglviewer->updateGL();
}

void Main_window::display_frame(int frame_id){
    if((frame_id<0) || !(frame_id<datastream->size())){
        mWarning("frame_id out of range: 0<=%d<%d\n",frame_id, datastream->size());
        fflush(stdout);
        return;
    }
    DataFrame& frame = datastream->get_frame(frame_id);
    update_slider(frame_id);
    update_sliderlabel(frame_id);
   
    if(solutions->isValid(frame_id)){
        std::vector<Scalar> theta = solutions->get(frame_id);
        if(theta.size()>0)
            worker->skeleton->set(theta);
    }

    ///--- If it's already done nothing happens anyway...
    worker->handfinder->binary_classification(frame);

    ///--- Update the current dataframe in the worker
    worker->current_frame = frame;
    
    ///--- Upload the new texture data (if necessary)
    worker->sensor_color_texture->load(frame.color.data, frame.id );
    worker->sensor_depth_texture->load(frame.depth.data, frame.id );
    qglviewer->updateGL();
}

void Main_window::save_slot(){ 
    QString filename = QFileDialog::getSaveFileName(QApplication::activeWindow(), tr("Save as..."), QDir::currentPath()+"/recs");
    datastream->save(filename);
}

void Main_window::load_slot(){ 
    QString filename = QFileDialog::getOpenFileName(QApplication::activeWindow(), tr("Load..."), QDir::currentPath()+"/recs");
    datastream->load(filename);

    ///--- Load default calibration
    Calibration(worker).autoload();
    qglviewer->reload_model();

    ///--- Load the first frame
    if(datastream->size()>0)
        display_frame(0);
}

void Main_window::save_frame_slot(){
    int fid = slider->value();
    DataFrame& frame = datastream->get_frame(fid);
    QString filename = QFileDialog::getSaveFileName(this, tr("Save as..."), QDir::currentPath());
    if( !filename.isNull() ){
        cv::Mat color_bgr;
        cv::cvtColor(frame.color, color_bgr, CV_RGB2BGR);
        bool s1 = cv::imwrite((filename+"_color.png").toStdString().c_str(), color_bgr);
        bool s2 = cv::imwrite((filename+"_depth.png").toStdString().c_str(), frame.depth);
        if(!s1 || !s2) LOG(INFO) << "somethign went wrong";
        LOG(INFO) << "frame# fid saved to files " << filename.toStdString() << "_{color,depth}.png";
        /// TEST (yes, opencv stores BGR instead of RGB)
        // QImage im1((uchar*) frame.color.data, stream->width(), stream->height(), QImage::Format_RGB888);
        // im1.save(filename+"_color2.png");
    }
}

void Main_window::save_calibration_slot(){ 
    QString filename = QFileDialog::getSaveFileName(QApplication::activeWindow(), tr("Save calibration as..."), QDir::currentPath()+"/calib");
    Calibration(worker).save(filename.toStdString());
}

void Main_window::load_calibration_slot(){ 
    QString filename = QFileDialog::getOpenFileName(QApplication::activeWindow(), tr("Load calibration..."), QDir::currentPath()+"/calib");
    Calibration(worker).load(filename.toStdString());
    qglviewer->reload_model();
    qglviewer->updateGL();
}

void Main_window::update_slider(int frame){
    slider->setRange(0,datastream->size());
    slider->setValue(frame);
}

void Main_window::update_sliderlabel(int frame_id){
    QString one_frame_idx = QString::number(frame_id);
    QString frames_size = QString::number(datastream->size()-1);
    slider_label->setText(one_frame_idx+"/"+frames_size);
}

//== hand calibration =============================================//

void Main_window::keyPressEvent(QKeyEvent *event){
    /// @todo this is __DUPLICATE CODE__ (TW application needs to do the same)
    switch(event->key()){
        case Qt::Key_Escape:
            this->close();
            break;
        case Qt::Key_1:
            // make_hand_thinner();
            worker->skeleton->scaleWidth(-5);
            Calibration::update_radius_helper(worker->cylinders, worker->skeleton);
            qglviewer->reload_model();
            break;
        case Qt::Key_2:
            // make_hand_wider();
            worker->skeleton->scaleWidth(5);
            Calibration::update_radius_helper(worker->cylinders, worker->skeleton);
            qglviewer->reload_model();
            break;
        case Qt::Key_3:
            // make_hand_shorter();
            worker->skeleton->scaleHeight(-1);
            Calibration::update_radius_helper(worker->cylinders, worker->skeleton);
            qglviewer->reload_model();
            break;
        case Qt::Key_4:
            // make_hand_longer();
            worker->skeleton->scaleHeight(1);
            Calibration::update_radius_helper(worker->cylinders, worker->skeleton);
            qglviewer->reload_model();
            break;
        case Qt::Key_5:
            // make_hand_smaller();
            worker->skeleton->scale(0.99f);
            qglviewer->reload_model();
            break;
        case Qt::Key_6:
            // make_hand_bigger();
            worker->skeleton->scale(1.01f);
            qglviewer->reload_model();
            break;
        default:
            QMainWindow::keyPressEvent(event);
    }
}
//=================================================================//

//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
//---------------------------               EXPERIMENTS        -------------------------------------
//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------

void Main_window::save_joint_position_list(){
    std::ofstream out("joint_positions.txt");
    for (int fid = 0; fid < datastream->num_frames(); fid++){
        ///--- apply parameters
        worker->skeleton->set( solutions->get(fid) );
        ///--- save joint positions to file
        for(int i=0; i<worker->skeleton->getNumberOfJoints(); i++){
            Joint * joint = worker->skeleton->getJoint(i);
            cout << joint->getName() << std::endl;
            Vector3 s = joint->getGlobalTranslation();            
            out << s.transpose() << " ";
        }
        out << std::endl;
    }
    out.close();
}

