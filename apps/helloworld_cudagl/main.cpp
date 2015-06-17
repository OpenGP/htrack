// Test the CUDA OpenGL interoperability
//
// Performance on GeForce 650M (OSX):
//   [cudaGraphicsGLRegisterImage] 21.8078 ms
//   [1000x (w/  offscreen draw)] 1687.07 ms
//   [1000x (w/o offscreen draw)] 1438.53 ms
//   [cudaGraphicsUnregisterResource] 0.555032 ms
// Performance on GeForce GTX980 and i5@3Ghz (Ubuntu@MIT):
//    [cudaGraphicsGLRegisterImage] 170.678 ms
//    [1000x (w/  offscreen draw)] 204.626 ms
//    [1000x (w/o offscreen draw)] 195.564 ms
//    [cudaGraphicsUnregisterResource] 0.12116 ms
// Performance on GeForce GTX980 (Win8.1):
//   [cudaGraphicsGLRegisterImage] 109.379 ms
//   [1000x (w/  offscreen draw)] 182.223 ms
//   [1000x (w/o offscreen draw)] 156.25 ms
//   [cudaGraphicsUnregisterResource] 0 ms

#include <iostream>
#include "util/gl_wrapper.h"
#include "util/tictoc.h"
#include "cudax/CudaHelper.h"
#include <QApplication>
#include <QGLWidget>
#include <QOpenGLVertexArrayObject>
#include "tracker/OffscreenRender/CustomFrameBuffer.h"

///--- Create an OpenGL>3 context
class GLWidget : public QGLWidget{
    QOpenGLVertexArrayObject vao;
    class OpenGLFormat : public QGLFormat{
    public:
        OpenGLFormat(){
            setVersion(3,2);
            setProfile(QGLFormat::CoreProfile);
            setSampleBuffers(false); ///< no anti-aliasing
            // setSamples(1); ///< no anti-aliasing
        }
    };
public:
    GLWidget():QGLWidget(OpenGLFormat()){
        std::cout << "OpenGL Context " << this->format().majorVersion() << "." << this->format().minorVersion() << std::endl;        
    }      
    void init(){
        bool success = vao.create();
        if(!success) exit(EXIT_FAILURE);
        vao.bind();        
    }
};

int main(int argc, char* argv[]){
    printf("%s starting...\n", argv[0]);
    QApplication app(argc, argv);
    GLWidget glarea;
    glarea.show();
    glarea.init();

#ifdef WITH_GLEW
	//--- GLEW Initialization (must have a context)
	glewExperimental = true;
	if (glewInit() != GLEW_NO_ERROR){

		fprintf(stderr, "Failed to initialize GLEW\n");
		// return EXIT_FAILURE;
	}
#endif

    /// As in de docs, I get the device after creating the context
    int devID = gpuGetMaxGflopsDeviceId();
    cudaError status = cudaGLSetGLDevice(devID);
    if(status!=cudaSuccess){
        std::cout << "Could not get OpenGL compliant device... exiting" << std::endl;
        exit(0);
    }
        
    ///--- These two are paired-up with a bind
    struct ResourcePair {
        struct cudaGraphicsResource* resouce;
        cudaArray* array;
    };

    ///--- OpenGL initialization
    int width = 320;
    int height = 240;
    glViewport(0,0, width, height);
    glClearColor(1.0,1.0,1.0,1.0);
    glEnable(GL_DEPTH_TEST);    
    
    ///--- Framebuffer initialization
    CustomFrameBuffer fb;
    fb.init(width, width);
    GLuint texture_id = fb.extra_tex_id();
    
    ResourcePair res;
    {
        TICTOC_SCOPE(timer, "cudaGraphicsGLRegisterImage");
        checkCudaErrors(cudaGraphicsGLRegisterImage(&res.resouce, texture_id, GL_TEXTURE_2D, cudaGraphicsMapFlagsReadOnly));
    }
    
    auto draw_map_unmap = [&](bool with_draw){
        if(with_draw){
            TIMED_SCOPE(timer,"offscreendraw");
            fb.bind();
                glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
            fb.unbind();
            glFinish();      
        }
        {    
            TIMED_SCOPE(timer,"map");
            checkCudaErrors(cudaGraphicsMapResources(1, &res.resouce, 0));
            checkCudaErrors(cudaGraphicsSubResourceGetMappedArray(&res.array, res.resouce, 0, 0));
        }        
        {
           TIMED_SCOPE(timer,"unmap"); //~2 microseconds
           checkCudaErrors(cudaGraphicsUnmapResources(1, &res.resouce, 0));        
        }
    };
    
    TICTOC_BLOCK(timer,"1000x (w/  offscreen draw)")
        for (int i = 0; i < 1000; ++i)
            draw_map_unmap(true);
    TICTOC_BLOCK(timer,"1000x (w/o offscreen draw)")
        for (int i = 0; i < 1000; ++i)
            draw_map_unmap(true);

    TICTOC_BLOCK(timer,"cudaGraphicsUnregisterResource")
    {
        checkCudaErrors(cudaGraphicsUnregisterResource(res.resouce));
    }
    return 0;
}
