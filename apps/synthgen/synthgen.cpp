#if __unix__
#define GL_GLEXT_PROTOTYPES 1
#include <GL/gl.h>
#include <GL/glext.h>
#endif

#include <QApplication>
#include <QSettings>

#include "DataStream.h"
#include "Sensor.h"
#include "Skeleton.h"
#include "Cylinders.h"
#include "Cylinders_renderer.h"
#include "PostureFile.h"
#include "CustomFrameBuffer.h"
#include "OpenGL32Format.h"
#include "mylogger.h"
#include "Types.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <iostream>
#include <iomanip>

_INITIALIZE_EASYLOGGINGPP

Settings* settings = NULL;
OpenGL_viewer* glarea = NULL;



int record_sequence(int argc, char* argv[]){
    setup_logger();
    QApplication app(argc, argv);

    string input  = "/Users/anastasia/Desktop/Data/Tompson/cpp_new/";
    QString output = "/Users/anastasia/Developer/htrack-data/recs/tompson_seq_2.dat";

    Settings _settings(QDir::currentPath()+"/settings.ini");
    settings = &_settings;

    DataStream stream(Tompson);

    int num_frames = 2440;

    for (int i = 0; i < num_frames; ++i) {

        char filename_suffix_char_array[9]; sprintf(filename_suffix_char_array, "1_%07d", i + 1);
        string filename_suffix(filename_suffix_char_array);

        string depth_image_name = input + "depth_" + filename_suffix + ".png";
        cv::Mat depth = cv::imread(depth_image_name, CV_LOAD_IMAGE_UNCHANGED);
        
        cout << i << endl;



        string color_image_name = input + "rgb_" + filename_suffix + ".png";
        cv::Mat color = cv::imread(color_image_name, CV_LOAD_IMAGE_UNCHANGED);

        /*cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );
        cv::imshow( "Display window", color );
        cv::waitKey(0);*/

        /*for(int row = 194; row < 200; row++){
            for(int col = 194; col < 200; col++){
                cout << depth.at<unsigned short>(row, col) << " ";
            }
            cout << "; " << endl;
        }
        cout << endl;*/

        if(!depth.data || !color.data ) {
            cout <<  "Could not open or find the images" << std::endl;
            return -1;
        }
        stream.add_frame(color.data, depth.data);
    }

    QFile::remove(output);
    stream.save(output);
    printf("%s finished!\n",argv[0]);
    fflush(stdout);
    return EXIT_SUCCESS;
}


/// @example this is generated programmatically
///     synthgen SIMPLE_WAVE ../htrack-data/recs/synthwave.dat
///     synthgen SIMPLE_MOVE_X ../htrack-data/recs/synthmovex.dat
///     synthgen SIMPLE_MOVE_Y ../htrack-data/recs/synthmovey.dat
///     synthgen SIMPLE_MOVE_Z ../htrack-data/recs/synthmovez_sphere.dat
///     synthgen SIMPLE_INDEX_SWING ../htrack-data/recs/synthmovez_sphere.dat
///
/// @example this reads postures from file and generate a stream
///     synthgen ./synthwave.mat ../htrack-data/recs/synthwave.dat
/// @example this reads every 100th posture from file and generates a stream
///     synthgen ./synthwave.mat ../htrack-data/recs/synthwave.dat 100
int main(int argc, char* argv[]){
    
    return record_sequence(argc, argv);
    
    setup_logger();
    if(argc==1){
        LOG(INFO) << "examples";
        LOG(INFO) << "./synthgen SIMPLE_INDEX_SWING ../htrack-data/recs/synth_index_swing.dat";
        LOG(INFO) << "./synthgen SIMPLE_MOVE_Z      ../htrack-data/recs/synthmovez.dat";
        LOG(INFO) << "./synthgen SIMPLE_COLLISION   ../htrack-data/recs/synth_collision.dat";
        LOG(INFO) << "./synthgen ../htrack-data/jangles/genMovements/20101220_152738_Trial02.mat ./recs/synth_genmov02.dat 10";
        exit(0);
    }

    printf("%s starting...\n\n", argv[0]);
    QApplication app(argc, argv);

    ///--- parse input
    CHECK(argc >= 3) << "wrong number of arguments";
    QString input  = argv[1];
    QString output = argv[2];
    LOG(INFO) << "input: " << argv[1];

    /// Settings
    Settings _settings(QDir::currentPath()+"/settings.ini");
    settings = &_settings;

    ///--- Read from "input" or generate programmatically
    std::vector< vector<float> > thetas;
    bool readFromFile = false;
    {

        if(!input.compare("THUMB_INDEX_COLLISION"))
        {
            const int num = 3;
            VectorN m_theta = VectorN::LinSpaced(num, -49, -50);
            float initial[20] = {0.192209, -0.550227, -0.277241, -0.611161, 0.2734, -0.868817, -1.55841, -0.749486,  0.1484,
                                 -0.71486, -1.50187, -0.93126, -0.0890169, -0.887832, -1.659, -1.00195, -0.230305, -1.12951,
                                 -0.577767, -1.82401};

            for (int i = 0; i < num; ++i) {
                vector<float> t(26, 0.0f);
                for (int j = 0; j < 20; ++j) {
                    t[j + 6] = initial[j];
                }
                // global translation
                t[0] =   0.0f;
                t[1] = -70.0f;
                t[2] = 400.0f;

                // collision route
                t[11] = m_theta[i] * M_PI / 180.0;
                thetas.push_back(t);
            }
        }

        else if(!input.compare("IMPROBABLE_POSE"))
        {
            const int num = 5;
            float initial[20] = {0.2245, 0.6083, -0.1260, -1.0965, -0.1580, -1.0081, -0.6572, -1.0159, -0.2056, 0.0670, -1.1924,
                                 -0.9529, -0.2882, -1.2361, -0.5992, -0.1336, 0.0659, -0.8790, -1.6130, -1.9549};

            for (int i = 0; i < num; ++i) {
                vector<float> t(num_thetas, 0.0f);
                for (int j = 0; j < 20; ++j) {
                    t[j + 6] = initial[j];
                }
                // global translation
                t[0] =   0.0f;
                t[1] = -70.0f;
                t[2] = 400.0f;

                thetas.push_back(t);
            }
        }


        else if(!input.compare("SIMPLE_COLLISION"))
        {
            const int num = 30;
            VectorN m_theta = VectorN::LinSpaced(num,0,-15);
            VectorN a_theta = VectorN::LinSpaced(num,0,+10);

            for (int i = 0; i < num; ++i) {
                vector<float> t(num_thetas, 0.0f);

                // global translation
                t[0] =   0.0f;
                t[1] = -10.0f;
                t[2] = 400.0f;

                // collision route
                t[18] = m_theta[i] * M_PI / 180.0;
                t[14] = a_theta[i] * M_PI / 180.0;
                thetas.push_back(t);
            }
        }

        else if(!input.compare("COLLISION_PROBLEM"))
        {
            const int num = 5;
            VectorN theta_10 = VectorN::LinSpaced(num,8,15);
            VectorN theta_11 = VectorN::LinSpaced(num,5,10);
            VectorN theta_14 = VectorN::LinSpaced(num,-8,-15);
            VectorN theta_15 = VectorN::LinSpaced(num,-5,-10);

            for (int i = 0; i < num; ++i) {
                vector<float> t(num_thetas, 0.0f);

                // global translation
                t[0] =   0.0f;
                t[1] = -10.0f;
                t[2] = 400.0f;

                // collision route
                t[10] = theta_10[i] * M_PI / 180.0;
                t[11] = theta_11[i] * M_PI / 180.0;
                t[14] = theta_14[i] * M_PI / 180.0;
                t[15] = theta_15[i] * M_PI / 180.0;
                thetas.push_back(t);
            }
        }

        else if(!input.compare("SIMPLE_WAVE"))
        {
            for(float a = 0; a < 45.0f; a += 0.5f)
            {
                vector<float> t(num_thetas, 0.0f);

                // global translation
                t[0] =   0.0f;
                t[1] = -10.0f;
                t[2] = 400.0f;

                // global rotation
                t[3] = 0.0f;
                t[4] = 0.0f;
                t[5] = a * M_PI / 180.0f;

                thetas.push_back(t);
            }
        }

        else if(!input.compare("SIMPLE_MOVE_Y"))
        {
            LOG(INFO) << "SIMPLE_MOVE_Y";
            for(float a = 0.0; a < 100.0f; a += 2.0f)
            {
                vector<float> t(num_thetas, 0.0f);

                // global translation
                t[0] =      0;
                t[1] =      a;
                t[2] = 400.0f;

                thetas.push_back(t);
            }
        }

        else if(!input.compare("SIMPLE_MOVE_X"))
        {
            Vector3 t0 = settings->vector3("skeleton/translation");

            for(float a = 0.0; a < 100.0f; a += 2.0f)
            {
                vector<float> t(num_thetas, 0.0f);

                // global translation
                t[0] = a+t0[0];
                t[1] =   t0[1];
                t[2] =   t0[2];

                thetas.push_back(t);
            }
        }

        else if(!input.compare("SIMPLE_MOVE_Z"))
        {
            for(float a = 200.0f; a > -200.0f; a -= 2.0f)
            {
                vector<float> t(num_thetas, 0.0f);

                // global translation
                t[0] =   0.0f;
                t[1] = -10.0f;
                t[2] = 400.0f + fabs(a);

                thetas.push_back(t);
            }
        }

        else if(!input.compare("SIMPLE_INDEX_SWING"))
        {
            for(float a = -10.0f; a < 25.0f; a += 1.0f)
            {
                vector<float> t(num_thetas, 0.0f);

                // global translation
                Vector3 t0 = settings->vector3("skeleton/translation");
                t[0] =   t0[0];
                t[1] =   t0[1];
                t[2] =   t0[2];

                // swing index left/right
                t[10] = -a * M_PI / 180.0f;

                thetas.push_back(t);
            }
        }

        else if(!input.compare("SIMPLE_MOVE_Z_2"))
        {
            for(float a = 0.0f; a > -200.0f; a -= 2.0f)
            {
                vector<float> t(num_thetas, 0.0f);

                // global translation
                t[0] =   0.0f;
                t[1] = -10.0f;
                t[2] = 400.0f + fabs(a);

                thetas.push_back(t);
            }
        }

        else if(!input.compare("SIMPLE_BEND_INDEX"))
        {
            for(float a = 0.0f; a < 100.0f; a += 1.0f)
            {
                vector<float> t(num_thetas, 0.0f);

                // global translation
                t[0] =   0.0f;
                t[1] = -10.0f;
                t[2] = 400.0f;

                // bend index finger
                t[11] = -a * M_PI / 180.0f;

                thetas.push_back(t);
            }
        }

        else
        {
            readFromFile = true;
            PostureFile p(argv[1]);

            if(argc >= 4)
            {
                int skip = atoi(argv[3]);

                for(size_t i = 0; i < p.postures.size(); i+=skip)
                {
                    vector<float> t = p.postures[i];

                    // global translation
                    Vector3 t0 = settings->vector3("skeleton/translation");
                    t[0] =   t0[0];
                    t[1] =   t0[1];
                    t[2] =   t0[2];

                    thetas.push_back(t);
                }
            }
            else
            {
                thetas = p.postures;
            }
        }
        LOG(INFO) << "rendering" << thetas.size() << "postures";
    }

    ///--- Where data will be saved
    DataStream stream;

    ///--- Initialize offscreen renderering context
    Camera camera(QVGA);
    OpenGL32Format format;
    QGLWidget glarea(format);
    glarea.makeCurrent();
    CustomFrameBuffer fb;
    glViewport(0,0,camera.width(), camera.height());
    glClearColor(1.0,1.0,1.0,1.0);
    glEnable(GL_DEPTH_TEST);
    fb.init(camera.width(), camera.height());

    ///--- Initialize renderer
    Skeleton *skeleton = Skeleton::leftHand();
    if(readFromFile){ ///<< ??????????????
        // this just looks better with the svn data
        skeleton->getJoint("Hand")->rotate(Vec3f(0,0,1), -1.57f);
        skeleton->setInitialTransformations();
    }
    Cylinders *cylinders = new Cylinders(skeleton);
    Cylinders_renderer renderer(cylinders);
    renderer.init(Cylinders_renderer::FRAMEBUFFER);
    renderer.set_uniform("view_projection", camera.view_projection_matrix());

    ///--- Prinout used matrices
    std::cout << "Using view_projection: \n " << camera.view_projection_matrix() << std::endl;

    cv::Mat color;
    cv::Mat xyz;
    for(auto t : thetas){
        skeleton->set(t);

        ///--- Render time
        fb.bind();
        glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
        renderer.render();
        fb.unbind();

        // #define JUST_SHOW_BUFFERS
#ifdef JUST_SHOW_BUFFERS
        fb.display_color_attachment();
        fb.display_extra_attachment();
        cv::waitKey(0);
        break;
#endif

        ///--- Fetch the textures
        fb.fetch_color_attachment(color);
        fb.fetch_extra_attachment(xyz);

        ///--- Flip them to match device
        cv::flip(color, color, 0 /*flip rows*/ );
        cv::flip(xyz,   xyz,   0 /*flip rows*/ );

        ///--- Convert to depth map
        cv::Mat synthdepth = cv::Mat(stream.height(), stream.width(), CV_16UC1, cv::Scalar(1.0));
        for(int row = 0; row < synthdepth.rows; row++)
            for(int col = 0; col < synthdepth.cols; col++){
                cv::Vec4f p = xyz.at<cv::Vec4f>(row, col);
                synthdepth.at<ushort>(row, col) = (ushort) p[2];
            }
        ///--- Convert IDs to an indexed color
        cv::Mat synthcolor = cv::Mat(stream.height(), stream.width(), CV_8UC4, cv::Scalar(255,255,255,255));
        cvtColor(color, synthcolor, CV_GRAY2RGB);

        ///--- Add synthetic data to stream
        stream.add_frame(synthcolor.data, synthdepth.data);
    }

    QFile::remove(output);
    stream.save(output);
    printf("%s finished!\n",argv[0]);
    fflush(stdout);
    return EXIT_SUCCESS;
}

