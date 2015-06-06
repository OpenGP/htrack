#pragma once
namespace opengl{

inline void print_projection_matrix()
{
    float m[16] = {0};
    glGetFloatv(GL_PROJECTION_MATRIX, m); 
    Eigen::Map<Eigen::Matrix4f> p(m);
    std::cout << "OpenGL Projection Matrix" << p << endl << std::endl;    
}
    
inline void print_modelview_matrix()
{
    float m[16] = {0};
    glGetFloatv(GL_MODELVIEW_MATRIX, m); 
    Eigen::Map<Eigen::Matrix4f> p(m);
    std::cout << "OpenGL ModelView Matrix" << endl << p << std::endl;    
}

inline void draw_unit_spiral()
{
  const float nbSteps = 200.0;
  glBegin(GL_QUAD_STRIP);
  for (int i=0; i<nbSteps; ++i)
	{
	  const float ratio = i/nbSteps;
	  const float angle = 21.0*ratio;
	  const float c = cos(angle);
	  const float s = sin(angle);
	  const float r1 = 1.0 - 0.8f*ratio;
	  const float r2 = 0.8f - 0.8f*ratio;
	  const float alt = ratio - 0.5f;
	  const float nor = 0.5f;
	  const float up = sqrt(1.0-nor*nor);
	  glColor3f(1.0-ratio, 0.2f , ratio);
	  glNormal3f(nor*c, up, nor*s);
	  glVertex3f(r1*c, alt, r1*s);
	  glVertex3f(r2*c, alt+0.05f, r2*s);
	}
  glEnd();
}

#if 0 /// not finished
/// @brief converts OpenGL [0,1] depth value to world-space depth value
/// @warning assumes a non-skewed projective transformation matrix
static float depth_to_depth(float depth){
    // Undo perspective division on the z-coordinate.
    // First we map the depth from [0, 1].
    float z_divided = depth * 2.0 - 1.0;
    
    // A default perspective projection transforms the z-coordinate as follows:
    // z_divided = (z * (f + n) + 2 * f * n) /  (z * (f - n))
    // (see http://www.songho.ca/opengl/gl_projectionmatrix.html for more info.)
    // Solving this equation for z leads to:
    // z = 2.0 * f * n / (z_divided * (f - n) - (f + n))
    float z = 2.0 * far * near / (z_divided * (far - near) - (far + near));
    return z;
}
    
///// Convert from OpenGL [0,1] format to OpenNI [0,..(mm)]
//cv::Mat depth_temp = cv::Mat(camera->size().height(), camera->size().width(), CV_16UC1, cv::Scalar(0));
//void convert(const cv::Mat& in, cv::Mat& out){
//    LOG_ASSERT((in.cols==out.cols) && (in.rows==out.rows),"size mismatch");
//    LOG_ASSERT((in.type()==CV_32FC1),"format invalid");
//    LOG_ASSERT((out.type()==CV_16UC1),"format invalid"); 
//    /// Convert
//    for(int y=0; y<in.rows; y++){
//        for(int x=0; x<in.cols; x++){  
//            in.at<unsigned char>(y,x) = (unsigned char) opengl::depth_to_depth(in.at<float>(y,x));
//        }
//    }
//}
#endif

} // opengl::


