#include "Fitting.h"

#include "util/mylogger.h"
#include "util/tictoc.h"

#include "tracker/Worker.h"
#include "tracker/HandFinder/HandFinder.h"
#include "tracker/Data/TextureDepth16UC1.h"
#include "tracker/DataStructure/SkeletonSerializer.h"
#include "tracker/OffscreenRender/CustomFrameBuffer.h" ///< TODO remove
#include "tracker/Legacy/geometry/Cylinders.h"
#include "tracker/TwSettings.h"

#include "Fitting/DistanceTransform.h"

#ifdef WITH_CUDA
#include "cudax/CudaHelper.h"
#include "cudax/CublasHelper.h"

#include <cuda_gl_interop.h>
struct MappedResource {
    struct cudaGraphicsResource* resouce = NULL;
    cudaArray* array = NULL;
    GLuint texid = 0;

    void init(GLuint texid){
        this->texid=texid;
        checkCudaErrors(cudaGraphicsGLRegisterImage(&resouce, texid, GL_TEXTURE_2D, cudaGraphicsMapFlagsReadOnly));
    }
    void cleanup(){
        checkCudaErrors(cudaGraphicsUnregisterResource(resouce));
    }

    cudaArray* bind() {
        checkCudaErrors(cudaGraphicsMapResources(1, &resouce, 0));
        checkCudaErrors(cudaGraphicsSubResourceGetMappedArray(&array, resouce, 0, 0));
        return array;
    }
    void unbind() {
        checkCudaErrors(cudaGraphicsUnmapResources(1, &resouce, 0));
    }
};

MappedResource render_color;
MappedResource render_xyz;
MappedResource render_normals;
MappedResource sensor_depth;

DistanceTransform distance_transform;

void energy::Fitting::cleanup() {
    kernel_cleanup(); ///< disposes of static resources

    render_color.cleanup();
    render_xyz.cleanup();
    render_normals.cleanup();
    sensor_depth.cleanup();
    distance_transform.cleanup();

    cudax::CublasHelper::cleanup();
    cudax::CudaHelper::cleanup();
}

void energy::Fitting::init(Worker *worker)
{
    this->camera = worker->camera;
    this->offscreenrend = &(worker->offscreenrend);
    this->sensor_depth_texture = worker->sensor_depth_texture;
    this->skeleton = worker->skeleton;
    this->cylinders = worker->cylinders;
    this->handfinder = worker->handfinder;

    ///--- 3D fitting
    tw_settings->tw_add(settings->fit3D_enable,"E_3D (enable)","group=Fitting");
    tw_settings->tw_add(settings->fit3D_weight,"E_3D (weight)","group=Fitting");
    tw_settings->tw_add(settings->fit3D_reweight,"E_3D (l1nrm?)","group=Fitting");
    tw_settings->tw_add(settings->fit3D_backface_check,"E_3D (occlus?)","group=Fitting");
    tw_settings->tw_add(settings->fit3D_point2plane,"E_3D (p2P?)","group=Fitting");
    ///--- 2D fitting
    tw_settings->tw_add(settings->fit2D_enable,"E_2D (enable)","group=Fitting");
    tw_settings->tw_add(settings->fit2D_weight,"E_2D (weight)","group=Fitting");

#ifdef WITH_CUDA
    cudax::CudaHelper::init();
    cudax::CublasHelper::init();

    ///--- Run some tests before we get started
    kernel_memory_tests();

    ///--- Init worker for GPU computation of normals
    distance_transform.init(camera->width(), camera->height());

    ///--- init resource mapper for cuda
    render_color.init(offscreenrend->fb->color_tex_id());
    render_xyz.init(offscreenrend->fb->extra_tex_id());
    render_normals.init(offscreenrend->fb->norms_tex_id());
    sensor_depth.init(sensor_depth_texture->texid());

    // LOG(INFO ) << camera->inv_projection_matrix();
    kernel_init(this->settings, camera->width(), camera->height(), num_thetas, camera->focal_length_x(), camera->focal_length_y(), camera->inv_projection_matrix().data());
    CHECK_ERROR_GL();
#endif
}

void energy::Fitting::track(DataFrame& frame, LinearSystem& sys, bool rigid_only, bool eval_error, float & push_error, float & pull_error) {
    // TICTOC_SCOPE(timer,"Energy::Fitting");

    ///--- Make sure sensor has necessary data
    assert( sensor_depth_texture->check_loaded(frame.id) );

    // TICTOC_BLOCK(timer,"Worker::track_cuda::(KinematicChainTransfer)")
    {
        kernel_upload_kinematic(skeleton->getJointTransformations(),skeleton->getKinematicChain());
        kernel_upload_cylinders(cylinders->serialize());
    }


    {
        cv::Mat& sensor_silhouette = handfinder->sensor_silhouette;
        static int last_uploaded_id=-1; ///< avoid multiple uploads
        static cv::Mat sensor_silhouette_flipped;
        if(last_uploaded_id!=frame.id){
            // TICTOC_SCOPE(t_dtform,"Energy::Fitting::dtform");
            cv::flip(sensor_silhouette, sensor_silhouette_flipped, 0 /*flip rows*/ );
            distance_transform.exec(sensor_silhouette_flipped.data, 125);
            kernel_upload_dtform_idxs(distance_transform.idxs_image_ptr());

            //---- WARNING THIS CORRUPTS DATA!!
            // cv::normalize(distance_transform.dsts_image(), distance_transform.dsts_image(), 0.0, 1.0, cv::NORM_MINMAX);
            // cv::imshow("dt", distance_transform.dsts_image());
            kernel_upload_sensor_data(sensor_silhouette_flipped.data);
            last_uploaded_id = frame.id;
        }
    }

    ///---------------------------------------------------
    ///---------------------------------------------------
    // cudaDeviceSynchronize();
    ///---------------------------------------------------
    ///---------------------------------------------------


    ///--- Map resources to CUDA context
    // TIMED_BLOCK(timer,"Worker::track_cuda::(bind+kernel)")
    {
        // TICTOC_BLOCK(timer,"Worker::track_cuda::(BindOpenGL)")
        {
            offscreenrend->fb->bind(); ///< with glFinish() only takes 20 microseconds
            cudax::render_color   = render_color.bind();
            cudax::render_points  = render_xyz.bind();
            cudax::sensor_depth   = sensor_depth.bind();
        }

        // TICTOC_BLOCK(timer,"Worker::track_cuda::(kernel)")
        {
            kernel_bind();
            bool reweight = settings->fit3D_reweight;
            if(rigid_only && settings->fit3D_reweight && !(settings->fit3D_reweight_rigid))
                reweight = false; ///< allows fast rigid motion (mostly visible on PrimeSense @60FPS)
            // std::cout << "rigid?" << rigid_only << "reweight?" << reweight << std::endl;
            kernel(sys.lhs.data(), sys.rhs.data(), push_error, pull_error, eval_error, reweight);
            kernel_unbind();
        }

        ///--- debug
        // std::ofstream("mat/JtJ_gpu.txt") << sys.lhs << std::endl;
        // std::ofstream("mat/Jte_gpu.txt") << sys.rhs << std::endl;

        if(settings->debug_show_constraints_image){
            int w = camera->height(), h = camera->width();
            cv::Mat opencv_image = cv::Mat(w, h, CV_8UC3, cv::Scalar(0,0,0));
            kernel_constraint_type_image(opencv_image.data, w, h);
            cv::flip(opencv_image, opencv_image, 0);
            cv::imshow("constraint types", opencv_image);
        }

        // TICTOC_BLOCK(timer,"Worker::track_cuda::(unbind)")
        {
            render_color.unbind();
            render_xyz.unbind();
            sensor_depth.unbind();
            offscreenrend->fb->unbind();
        }
    }

    ///--- @note debug
    // cv::imshow("debug_image", debug_image);
}
#endif
