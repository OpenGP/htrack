#pragma once
#ifdef WITH_OPENCV
    #include "opencv2/core/core.hpp"    
#endif

class DistanceTransform{
private:
    int width;
    int height;
    float* v=NULL;
    float* z=NULL;
    float* DTTps=NULL;
    int* ADTTps=NULL;
    float* realDT=NULL;
    int* realADT=NULL; ///< stores closest point ids (float just to simplify CUDA)
    
public:
    void init(int width, int height){
        this->width = width;
        this->height = height;
        v = new float[width*height];
        z = new float[width*height];
        DTTps = new float[width*height];
        ADTTps = new int[width*height];
        realADT = new int[width*height];
        realDT = new float[width*height];
    }
    void cleanup(){
        delete[] v;
        delete[] z;
        delete[] DTTps;
        delete[] ADTTps;
        delete[] realADT;
        delete[] realDT;        
    }
    
#ifdef WITH_OPENCV
    float* dsts_image_ptr(){ return realDT; }
    int* idxs_image_ptr(){ return realADT; }
    cv::Mat dsts_image(){ return cv::Mat(height, width, CV_32FC1 /*float*/, realDT); }
    cv::Mat idxs_image(){ return cv::Mat(height, width, CV_32SC1 /*int*/, realADT); }
    int dst_at(int row, int col){ return realDT[row*width+col]; }
    int idx_at(int row, int col){ return realADT[row*width+col]; }
#endif
    
public:
    /// @pars row major uchar binary image. White pixels are "data" and 
    /// label_image[i] > mask_th decides what data is.
    void exec(unsigned char* label_image, int mask_th=125)
    {
//        #pragma omp parallel
        {
//            #pragma omp for
            for(int i = 0; i < width*height; ++i)
            {
                if(label_image[i]<mask_th)
                    realDT[i] = FLT_MAX;
                else
                    realDT[i] = 0.0f;
            }

            /////////////////////////////////////////////////////////////////
            /// DT and ADT
            /////////////////////////////////////////////////////////////////

            //First PASS (rows)
//            #pragma omp for
            for(int row = 0; row<height; ++row)
            {
                unsigned int k = 0;
                unsigned int indexpt1 = row*width;
                v[indexpt1] = 0;
                z[indexpt1] = FLT_MIN;
                z[indexpt1 + 1] = FLT_MAX;
                for(int q = 1; q<width; ++q)
                {
                    float sp1 = float(realDT[(indexpt1 + q)] + (q*q));
                    unsigned int index2 = indexpt1 + k;
                    unsigned int vk = v[index2];
                    float s = (sp1 - float(realDT[(indexpt1 + vk)] + (vk*vk)))/float((q-vk) << 1);
                    while(s <= z[index2] && k > 0)
                    {
                        k--;
                        index2 = indexpt1 + k;
                        vk = v[index2];
                        s = (sp1 - float(realDT[(indexpt1 + vk)] + (vk*vk)))/float((q-vk) << 1);
                    }
                    k++;
                    index2 = indexpt1 + k;
                    v[index2] = q;
                    z[index2] = s;
                    z[index2+1] = FLT_MAX;
                }
                k = 0;
                for(int q = 0; q<width; ++q)
                {
                    while(z[indexpt1 + k+1]<q)
                        k++;
                    unsigned int index2 = indexpt1 + k;
                    unsigned int vk = v[index2];
                    float tp1 =  float(q) - float(vk);
                    DTTps[indexpt1 + q] = tp1*tp1 + float(realDT[(indexpt1 + vk)]);
                    ADTTps[indexpt1 + q] = indexpt1 + vk;
                }
            }

            //--- Second PASS (columns)
//            #pragma omp for
            for(int col = 0; col<width; ++col)
            {
                unsigned int k = 0;
                unsigned int indexpt1 = col*height;
                v[indexpt1] = 0;
                z[indexpt1] = FLT_MIN;
                z[indexpt1 + 1] = FLT_MAX;
                for(int row = 1; row<height; ++row)
                {
                    float sp1 = float(DTTps[col + row*width] + (row*row));
                    unsigned int index2 = indexpt1 + k;
                    unsigned int vk = v[index2];
                    float s = (sp1 - float(DTTps[col + vk*width] + (vk*vk)))/float((row-vk) << 1);
                    while(s <= z[index2] && k > 0)
                    {
                        k--;
                        index2 = indexpt1 + k;
                        vk = v[index2];
                        s = (sp1 - float(DTTps[col + vk*width] + (vk*vk)))/float((row-vk) << 1);
                    }
                    k++;
                    index2 = indexpt1 + k;
                    v[index2] = row;
                    z[index2] = s;
                    z[index2+1] = FLT_MAX;
                }
                k = 0;
                for(int row = 0; row<height; ++row)
                {
                    while(z[indexpt1 + k+1]<row)
                        k++;
                    unsigned int index2 = indexpt1 + k;
                    unsigned int vk = v[index2];
                    #ifdef ENABLE_DTFORM_DSTS
                        /// Also compute the distance value
                        float tp1 =  float(row) - float(vk);
                        realDT[col + row*width] = sqrtf(tp1*tp1 + DTTps[col + vk*width]);
                    #endif
                    realADT[col + row*width] = ADTTps[col + vk*width];
                }
            }
        } ///< OPENMP
    }
};


