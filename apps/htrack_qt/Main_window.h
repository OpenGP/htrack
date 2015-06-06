#pragma once
#include <QMainWindow>
#include <QLayout>
#include <QLabel>
#include <QSlider>
#include <QPushButton>
#include <QMenuBar>
#include <QKeyEvent>
#include <QTimer>
#include <QApplication>
#include "tracker/ForwardDeclarations.h"

class OpenGL_viewer;

class Main_window : public QMainWindow{
    Q_OBJECT
private:
    Worker*const worker;
    OpenGL_viewer*const qglviewer;
    DataStream*const datastream;
    SolutionStream*const solutions;
    QSlider* slider;
    QLabel* slider_label;
public:
    QPushButton* toggle_record_;
    QPushButton* toggle_live_;

public:
    ~Main_window(){ /*LOG(INFO) << __FUNCTION__;*/ }
    Main_window(Worker *worker, DataStream* datastream, SolutionStream* solutions, OpenGL_viewer* qglviewer);

public slots:
    void reinit_from_previous_frame();
    void requested_track(int);
    void requested_track_till_convergence(int);
    void track_whole_sequence(bool checked);
    void set_current_solution(int);
    void replay_sequence(bool checked);

public slots:
    void display_frame(int frame_id);
    void display_latest_frame();
    void display_next_frame();
    void display_previous_frame();
    void crop(int from, int to);
    void save_tracking_pressed();
    void load_tracking_pressed();
    void save_results_pressed();
    void save_results_single_pressed();
/// @{ actions
private:
    QAction* A_track_whole = NULL;
    QAction* A_replay = NULL;
/// @}

/// @{
private slots:
    void menu_track_pressed(){ emit requested_track(slider->value());}
    void menu_track_till_convergence_pressed(){ emit requested_track_till_convergence(slider->value()); }
    void menu_initialize_offset();
    void menu_set_current_solution(){ emit set_current_solution(slider->value()); };
/// @}


private slots:
    void save_slot();
    void load_slot();
    void save_frame_slot();
    void save_calibration_slot();
    void load_calibration_slot();
    void save_joint_position_list();
private:
    void update_gui(int frame_id);
    void update_slider(int frame);
    void update_sliderlabel(int frame_id);

private:
    void keyPressEvent(QKeyEvent *event);
};
