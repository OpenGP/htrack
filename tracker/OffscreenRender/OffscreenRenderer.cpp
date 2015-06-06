#include "OffscreenRenderer.h"

#include "util/tictoc.h"
#include "util/gl_wrapper.h"
#include "util/mylogger.h"
#include "tracker/Data/Camera.h"
#include "tracker/Legacy/geometry/Cylinders.h"
#include "tracker/OpenGL/CylindersRenderer/Cylinders_renderer.h"
#include "tracker/OffscreenRender/CustomFrameBuffer.h"

void OffscreenRenderer::init(Camera* camera, Cylinders *cylinders)
{
    this->camera = camera;
    this->cylinders = cylinders;
    fb = new CustomFrameBuffer(camera->width(), camera->height());
}

void OffscreenRenderer::render_offscreen(bool reinit){
#if 0
    ///--- TODO: Lazy resource allocation
    if(fb==NULL)
        fb = new CustomFrameBuffer(camera->width(), camera->height());
#endif

    // TICTOC_SCOPE(timer,"Worker::render_offscreen()");
    static Cylinders_renderer renderer(cylinders, Cylinders_renderer::FRAMEBUFFER, camera->view_projection_matrix());
    {
        if(reinit)
            renderer.init_geometry();
        glViewport(0,0,camera->width(), camera->height());
        glClearColor(1.0,1.0,1.0,1.0);
        glDisable(GL_BLEND); ///< just in case
        glEnable(GL_DEPTH_TEST);
        fb->bind();
            glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
            renderer.render();
        fb->unbind();
        glFinish();
    }
}
