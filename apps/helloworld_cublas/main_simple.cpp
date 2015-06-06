///--- Computes matrix products on GPU
/// this example is broken, but if it runs it's "okay"
/// TODO: matthias, it runs but frobenius is non-zero... why?

#include <cassert>
#include "util/gl_wrapper.h"
#include "util/OpenGL32Format.h"
#include <QApplication>
#include <QGLWidget>
#include <Eigen/Dense>
#include "cudax/CublasHelper.h"
#include "cudax/CudaHelper.h"
#include "cudax/CublasHelper.h"

using namespace std;
using namespace cudax;

typedef float Scalar;
typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Matrix_MxN;
extern "C" void outer_product(float* input, float* output, int rows, int cols);

int main(int argc, char *argv[]){
    QApplication app(argc, argv); 
    OpenGL32Format fmt;
    QGLWidget widget(fmt);
    widget.makeCurrent();
    CudaHelper::init();
    CublasHelper::init();
       
    {
        Matrix_MxN J = Matrix_MxN::Random(100,10);
        Matrix_MxN JtJ(J.cols(),J.cols());    
        outer_product(J.data(), JtJ.data(), J.rows(), J.cols());
        cout << J.transpose() * J << endl;
        cout << endl << endl;
        cout << JtJ << endl;
        cout << endl << endl;
        cout << "Frobenius norm: " << (J.transpose()*J - JtJ).norm() << endl;
    }
    
    CublasHelper::cleanup();
    CudaHelper::cleanup();
    return 0;
}
