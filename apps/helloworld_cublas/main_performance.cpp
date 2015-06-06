#include <cassert>
#include <Eigen/Dense>
#include <QApplication>
#include "util/gl_wrapper.h"
#include "util/OpenGL32Format.h"
#include <QGLWidget>
#include "thrust/device_vector.h"
#include "thrust/host_vector.h"
#include "thrust/copy.h"
#include "cudax/CublasHelper.h"
#include "cudax/CudaHelper.h"
#include "cudax/CudaTimer.h"
#include "./gnuplot_i.h"

using namespace std;
using namespace cudax;

typedef float Scalar;
typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Matrix_MxN;

extern "C" void outer_product(float* input, float* output, int rows, int cols);
extern "C" void vector_product(float* J_in, float* e_in, float* Jte_out, int rows, int cols);

//--

extern "C" void outer_product_init(float* input, int rows, int cols);
extern "C" void outer_product_compute(int rows, int cols);
extern "C" void outer_product_copy(float* output);

extern "C" void vector_product_init(float* J_in, float* e_in, int rows, int cols);
extern "C" void vector_product_compute(int rows, int cols);
extern "C" void vector_product_copy(float* Jte_out);

//--

void wait_for_key (){
    cout << endl << "Press ENTER to continue..." << endl;
    std::cin.clear();
    std::cin.ignore(std::cin.rdbuf()->in_avail());
    std::cin.get();
    return;
}

int main(int argc, char *argv[]){

    QApplication app(argc, argv);
    OpenGL32Format fmt;
    QGLWidget widget(fmt);
    widget.makeCurrent();

    CudaHelper::init();
    CublasHelper::init();    
    
    ///--- Jacobian multiplication
    {
        // Matrix dimensions
        const int MAX_NUM_CONSTRAINTS = 320*240;
        const int NUM_ROWS = 100;
        const int NUM_COLS = 26; ///< #DOF 1xhand

        // Row major matrix type
        typedef Eigen::Matrix<Scalar,
                Eigen::Dynamic,
                Eigen::Dynamic,
                Eigen::RowMajor> Matrix_MxN_RowMajor;

        //--- JTJ computation

        // Large matrix with MAX_NUM_CONSTRAINTS rows
        Matrix_MxN_RowMajor J;
        J = Matrix_MxN_RowMajor::Random(MAX_NUM_CONSTRAINTS, NUM_COLS);

        // GPU matrix product of the top NUM_ROWS rows
        cout << "compute:" << endl;
        Matrix_MxN JtJ_gpu(NUM_COLS, NUM_COLS);

#if 1
        outer_product_init(J.data(), NUM_ROWS, NUM_COLS);
        outer_product_compute(NUM_ROWS, NUM_COLS);
        outer_product_copy(JtJ_gpu.data());
#else
        outer_product(J.data(), JtJ_gpu.data(), NUM_ROWS, NUM_COLS);
#endif

        // CPU matrix product of the top NUM_ROWS rows
        Matrix_MxN J_sub = J.block(0, 0, NUM_ROWS, NUM_COLS);
        Matrix_MxN JtJ_cpu = J_sub.transpose() * J_sub;

        // Print results
        cout << "CPU result:" << endl << JtJ_cpu << endl << endl;
        cout << "GPU result:" << endl << JtJ_gpu << endl << endl;
        cout << "Frobenius norm: " << (JtJ_cpu - JtJ_gpu).norm() << endl << endl; 
        
        //--- JTe computation

        // Right hand side matrix-vector multiplication
        Matrix_MxN_RowMajor e;
        e = Matrix_MxN_RowMajor::Random(MAX_NUM_CONSTRAINTS, 1);
        Matrix_MxN e_sub = e.block(0, 0, NUM_ROWS, 1);

        // GPU matrix-vector product
        Matrix_MxN Jte_gpu(NUM_COLS, 1);

#if 1
        vector_product_init(J.data(), e.data(), NUM_ROWS, NUM_COLS);
        vector_product_compute(NUM_ROWS, NUM_COLS);
        vector_product_copy(Jte_gpu.data());
#else
        vector_product(J.data(), e.data(), Jte_gpu.data(), NUM_ROWS, NUM_COLS);
#endif

        // CPU matrix-vector product
        Matrix_MxN Jte_cpu = J_sub.transpose() * e_sub;

        // Print results
        cout << "CPU result:" << endl << Jte_cpu << endl << endl;
        cout << "GPU result:" << endl << Jte_gpu << endl << endl;
        cout << "Frobenius norm: " << (Jte_cpu - Jte_gpu).norm() << endl << endl;

        //--- Runtimes

        // Plot computation time
        vector<float> data_x0(MAX_NUM_CONSTRAINTS); // Matrix dimension
        vector<float> data_y1(MAX_NUM_CONSTRAINTS); // GPU JTJ runtime
        vector<float> data_y2(MAX_NUM_CONSTRAINTS); // CPU JTJ runtime
        vector<float> data_y3(MAX_NUM_CONSTRAINTS); // GPU JTe runtime
        vector<float> data_y4(MAX_NUM_CONSTRAINTS); // CPU JTe runtime

        CudaTimer timer;

        for(int i = 0; i < MAX_NUM_CONSTRAINTS; i += 100){
            int N = i+1;
            float x = (float)N;

            //--- JTJ

            Matrix_MxN_RowMajor J_test = J.block(0,0,N,NUM_COLS);
            Matrix_MxN JtJ_test(NUM_COLS, NUM_COLS);

            // GPU
            outer_product_init(J_test.data(), N, NUM_COLS);
            timer.restart("");
            outer_product_compute(N, NUM_COLS);
            //outer_product(J_test.data(), JtJ_test.data(), N, NUM_COLS);
            float y1 = timer.elapsed();
            outer_product_copy(JtJ_test.data());

            // CPU
            timer.restart("");
            Matrix_MxN JtJ_test_cpu = J_test.transpose() * J_test;
            float y2 = timer.elapsed();

            //cout << "Frobenius norm: " << (JtJ_test - JtJ_test_cpu).norm() << endl << endl;

            //--- JTe

            Matrix_MxN_RowMajor e_test = e.block(0,0,N,1);
            Matrix_MxN Jte_test(NUM_COLS, 1);

            // GPU
            vector_product_init(J_test.data(), e_test.data(), N, NUM_COLS);
            timer.restart("");
            vector_product_compute(N, NUM_COLS);
            //vector_product(J_test.data(), e_test.data(), Jte_test.data(), N, NUM_COLS);
            float y3 = timer.elapsed();
            vector_product_copy(Jte_test.data());

            // CPU
            timer.restart("");
            Matrix_MxN Jte_test_cpu = J_test.transpose() * e_test;
            float y4 = timer.elapsed();

            //cout << "Frobenius norm: " << (Jte_test - Jte_test_cpu).norm() << endl << endl;

            data_x0.push_back(x);
            data_y1.push_back(y1);
            data_y2.push_back(y2);
            data_y3.push_back(y3);
            data_y4.push_back(y4);
        }

        try{
            Gnuplot::set_terminal_std("qt");
            Gnuplot plot("lines");
            plot.set_title("Computation time of J^T * J w.r.t. number of rows of J in milliseconds");
            plot.set_grid();
            plot.plot_xy(data_x0, data_y1, "GPU JTJ");
            plot.plot_xy(data_x0, data_y2, "CPU JTJ");
            plot.plot_xy(data_x0, data_y3, "GPU JTe");
            plot.plot_xy(data_x0, data_y4, "CPU JTe");

            cout << "Computation time for "
                 << MAX_NUM_CONSTRAINTS << " x "
                 << NUM_COLS << " matrix: "
                 << data_y1[data_y1.size()-1] << " ms [GPU], "
                 << data_y2[data_y2.size()-1] << " ms [CPU]"
                 << endl;

            wait_for_key(); ///< avoid window closing right away
        }catch(GnuplotException &e){
            cout << e.what() << endl;
        }
    }

    CublasHelper::cleanup();
    exit(0);

    return 0;
}


