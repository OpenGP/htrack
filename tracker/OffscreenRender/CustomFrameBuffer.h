#pragma once
#include "util/gl_wrapper.h"

#include "util/mylogger.h"
#include <cassert>

#ifdef WITH_OPENCV
#include "opencv2/core/core.hpp"       ///< cv::Mat
#include "opencv2/highgui/highgui.hpp" ///< cv::imShow
#include "opencv2/contrib/contrib.hpp" ///< cvtConvert
#include "opencv2/imgproc/types_c.h"   ///< CV_BGRA2RGBA
#endif

class CustomFrameBuffer {
private:
    int image_width;
    int image_height;
    GLuint framebuffer; // to bind the proper targets
    GLuint depth_rb;    // for proper depth test while rendering the scene
    GLuint color_tex;   // where we render the face indices
    GLuint extra_tex;   // where we render screen space positions
    GLuint norms_tex;   // rendered model normals (screen space)
    bool  needs_cleanup;
public:
    CustomFrameBuffer():needs_cleanup(false) {}
    /// Constructor for static inline initialization
    CustomFrameBuffer(int image_width, int image_height){
        needs_cleanup = false;
        init(image_width, image_height);
    }
    ~CustomFrameBuffer() { if (needs_cleanup) cleanup(); }

public:
    GLuint color_tex_id() { return color_tex; }
    GLuint extra_tex_id() { return extra_tex; }
    GLuint norms_tex_id() { return norms_tex; }

public:
    /// Ready to be bound?
    bool ready() {
        return needs_cleanup;
    }
    void bind() {
        glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);
        const GLenum buffers[] = { GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1, GL_COLOR_ATTACHMENT2 };
        glDrawBuffers(3 /*length of list above*/, buffers); ///< TODO check if this needs
    }
    void unbind() {
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }

    void init(int image_width, int image_height) {
        CHECK(needs_cleanup==false);
        this->image_width = image_width;
        this->image_height = image_height;
        color_tex = create_color_attachment(image_width, image_height);
        depth_rb  = create_depth_attachment(image_width, image_height);
        extra_tex = create_extra_attachment(image_width, image_height);
        norms_tex = create_normals_attachment(image_width, image_height);        
        framebuffer = create_framebuffer();
        needs_cleanup = true;
    }

    void cleanup() {
        assert(needs_cleanup==true);
        //Delete resources
        glDeleteTextures(1, &color_tex);
        glDeleteTextures(1, &extra_tex);
        glDeleteTextures(1, &norms_tex);
        glDeleteRenderbuffersEXT(1, &depth_rb);
        
        //Bind 0, which means render to back buffer, as a result, fb is unbound
        glBindFramebuffer(GL_FRAMEBUFFER_EXT, 0);
        glDeleteFramebuffers(1, &framebuffer);
        needs_cleanup = false;
    }

private:
    GLuint create_color_attachment(int image_width, int image_height) {
        GLuint tex_screen;
        glGenTextures(1, &tex_screen);
        glBindTexture(GL_TEXTURE_2D, tex_screen);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        /// See Table.2 https://www.khronos.org/opengles/sdk/docs/man3/docbook4/xhtml/glTexImage2D.xml
        glTexImage2D(GL_TEXTURE_2D, 0, GL_R8UI, image_width, image_height, 0, GL_RED_INTEGER, GL_UNSIGNED_BYTE, NULL);
        return tex_screen;
    }
public:
    
#ifdef WITH_OPENCV
    cv::Mat& fetch_norms_attachment(){
        static cv::Mat image = cv::Mat(image_height, image_width, CV_32FC4, cv::Scalar(0));
        glBindTexture(GL_TEXTURE_2D, norms_tex);
        glGetTexImage(GL_TEXTURE_2D, 0, GL_RGBA, GL_FLOAT, image.data);
        glBindTexture(GL_TEXTURE_2D, 0);
        return image;
    }   
#endif
    
#ifdef WITH_OPENCV 
    cv::Mat& fetch_color_attachment(){
        static cv::Mat image = cv::Mat(image_height, image_width, CV_8UC1, cv::Scalar(0));
        fetch_color_attachment(image);
        return image;
    }
    void fetch_color_attachment(cv::Mat& image){
        if( image.empty() )
            image = cv::Mat(image_height, image_width, CV_8UC1, cv::Scalar(0));
        glBindTexture(GL_TEXTURE_2D, color_tex);
        glGetTexImage(GL_TEXTURE_2D, 0, GL_RED_INTEGER, GL_UNSIGNED_BYTE, image.data);
        glBindTexture(GL_TEXTURE_2D, 0);
    }
    void display_color_attachment() {
        static cv::Mat image;
        fetch_color_attachment(image);
        ///--- Postprocess
        cv::flip(image, image, 0 /*flip rows*/ ); ///< OpenGL / OpenCV top left origin
        cv::imshow("color_channel", image);
    }
#endif

    GLuint create_depth_attachment(int image_width, int image_height) {
        GLuint depth;
        glGenRenderbuffers(1, &depth);
        glBindRenderbuffer(GL_RENDERBUFFER_EXT, depth);
        glRenderbufferStorage(GL_RENDERBUFFER_EXT, GL_DEPTH_COMPONENT32, image_width, image_height);
        glBindRenderbuffer(GL_RENDERBUFFER_EXT, 0);
        // CHECK_ERROR_GL();
        return depth;
    }

    GLuint create_extra_attachment(int image_width, int image_height) {
        GLuint texture;
        glGenTextures(1, &texture);
        glBindTexture(GL_TEXTURE_2D, texture);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        /// See Table.2 https://www.khronos.org/opengles/sdk/docs/man3/docbook4/xhtml/glTexImage2D.xml
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, image_width, image_height, 0, GL_RGBA, GL_FLOAT, NULL);
        return texture;
    }
    
    GLuint create_normals_attachment(int image_width, int image_height) {
        GLuint texture;
        glGenTextures(1, &texture);
        glBindTexture(GL_TEXTURE_2D, texture);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        /// See Table.2 https://www.khronos.org/opengles/sdk/docs/man3/docbook4/xhtml/glTexImage2D.xml
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, image_width, image_height, 0, GL_RGBA, GL_FLOAT, NULL);
        return texture;
    }
    
    
#ifdef WITH_OPENCV
public:    
    cv::Mat& fetch_extra_attachment(){
        static cv::Mat image = cv::Mat(image_height, image_width, CV_32FC4, cv::Scalar(0));
        fetch_extra_attachment(image);
        return image;
    }
    void fetch_extra_attachment(cv::Mat& image){
        #ifdef WITH_OPENCV
        if( image.empty() )
            image = cv::Mat(image_height, image_width, CV_32FC4, cv::Scalar(0));
        glBindTexture(GL_TEXTURE_2D, extra_tex);
        glGetTexImage(GL_TEXTURE_2D, 0, GL_RGBA, GL_FLOAT, image.data);
        glBindTexture(GL_TEXTURE_2D, 0);
        #endif
    }
    void display_extra_attachment() {
        #ifdef WITH_OPENCV
        static cv::Mat image;
        fetch_extra_attachment(image);
        ///--- postprocess
        cv::flip(image, image, 0 /*flip rows*/ ); ///< OpenGL / OpenCV top left origin
        cv::cvtColor(image, image, CV_BGRA2RGBA); ///< TODO: not sure this is the right conversion...
        cv::imshow("extra_channel", image);
        #endif
    }
#endif

private:
    GLuint create_framebuffer() {
        GLuint fbo;
        // create and bind a framebuffer
        glGenFramebuffers(1, &fbo);
        glBindFramebuffer(GL_FRAMEBUFFER_EXT, fbo);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0 /*location = 0*/, GL_TEXTURE_2D, color_tex, 0 /*level*/);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1 /*location = 1*/, GL_TEXTURE_2D, extra_tex, 0 /*level*/);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT2 /*location = 2*/, GL_TEXTURE_2D, norms_tex, 0 /*level*/);
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depth_rb);
        if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
            std::cerr << "Framebuffer not OK." << std::endl;
        glBindFramebuffer(GL_FRAMEBUFFER, 0); ///< avoid pollution
        return fbo;
    }
};
