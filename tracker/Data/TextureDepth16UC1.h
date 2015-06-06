#pragma once
#include "util/gl_wrapper.h"
#include "util/mylogger.h"
#include "stdint.h" ///< uint16_t

class DepthTexture16UC1 {
private:
    GLuint texture = 0;
    int width, height;
    bool _is_init = false;
    int fid = -1; ///< loaded frame data (avoid unnecessary loads)
public:
    GLuint texid(){ CHECK(_is_init); return texture; }
    
    DepthTexture16UC1(int width, int height) {
        this->width = width;
        this->height = height;
        CHECK(!_is_init);
        glGenTextures(1, &texture);
        glBindTexture(GL_TEXTURE_2D, texture);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        std::vector<uint16_t> data(width*height, 0); ///< initialization        
        /// @see Table.2 https://www.khronos.org/opengles/sdk/docs/man3/docbook4/xhtml/glTexImage2D.xml
        glTexImage2D(GL_TEXTURE_2D, 0, GL_R16UI, width, height, 0, GL_RED_INTEGER, GL_UNSIGNED_SHORT, data.data());
        glBindTexture(GL_TEXTURE_2D, 0);
        _is_init=true;
    }
    
    /// @note Pixels must be a 16 bits unsigned short data!!!
    /// @see http://stackoverflow.com/questions/9863969/updating-a-texture-in-opengl-with-glteximage2d
    void load(GLvoid* pixels, int fid){
        if(this->fid != fid || fid==-1){
            glBindTexture(GL_TEXTURE_2D, texture);
            glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width, height, GL_RED_INTEGER, GL_UNSIGNED_SHORT, pixels);
            glBindTexture(GL_TEXTURE_2D, 0);
            this->fid = fid;
        }
    }
    
    bool check_loaded(int fid){ return this->fid == fid; }
     
#if 0
        cudaChannelFormatDesc channelDesc = cudaCreateChannelDesc(32, 0, 0, 0, cudaChannelFormatKindUnsigned);        
        cudaArray* myArray;
        cudaMallocArray(&myArray,
                        &refTex.channelDesc, /* with this you don't need to fill a channel descriptor */
                        width,
                        height);
    }
#endif
};
