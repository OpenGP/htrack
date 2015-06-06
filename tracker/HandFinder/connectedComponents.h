//Snipped of opencv3 imgproc.hpp containing these functions

#include "opencv2/core/core.hpp"

/// andrea: also on linux, this file does not exist!!!
#if 0 // !defined(__APPLE__)
    #include "opencv2/imgproc.hpp" ///< declares the elements below, prevent re-declaration
#else

namespace cv{

//! connected components algorithm output formats
enum { CC_STAT_LEFT   = 0,
       CC_STAT_TOP    = 1,
       CC_STAT_WIDTH  = 2,
       CC_STAT_HEIGHT = 3,
       CC_STAT_AREA   = 4,
       CC_STAT_MAX    = 5
     };

// computes the connected components labeled image of boolean image ``image``
// with 4 or 8 way connectivity - returns N, the total
// number of labels [0, N-1] where 0 represents the background label.
// ltype specifies the output label image type, an important
// consideration based on the total number of labels or
// alternatively the total number of pixels in the source image.
int connectedComponents(InputArray image, OutputArray labels,
                        int connectivity = 8, int ltype = CV_32S);

int connectedComponentsWithStats(InputArray image, OutputArray labels,
                                 OutputArray stats, OutputArray centroids,
                                 int connectivity = 8, int ltype = CV_32S);

} // cv::

#endif

