#pragma once
#include "tracker/Types.h"
#include <QSize>

/// The running modes for cameras
enum CAMERAMODE{ INVALID=-1, QVGA, VGA, Tompson, Chen, Tang, Intel};

class Camera{
    typedef Eigen::Matrix<float,3,3> Matrix33f;
    typedef Eigen::Matrix<float,4,4> Matrix44f;
private:
    CAMERAMODE _mode;
    Matrix3 proj;
    Matrix3 iproj;
    Scalar _zNear; ///< near clip plane (mm)
    Scalar _zFar;  ///< far clip plane (mm)
    int _width;
    int _height;
    Scalar _focal_length_x=nan();
    Scalar _focal_length_y=nan();
    int _fps=30; ///< frame per second
public:
    
/// @{
public:
    Camera(CAMERAMODE mode, int FPS=30);
public:
    CAMERAMODE mode() const { return _mode; }
    int width() const { return _width; }
    int height() const { return _height; }
    Scalar focal_length_x() const{ return _focal_length_x; }
    Scalar focal_length_y() const{ return _focal_length_y; }
    Scalar zSpan() const { return (_zFar-_zNear); }
    Scalar zNear() const { return _zNear; }
    Scalar zFar() const { return _zFar; }
    bool is_valid(Scalar depth){ return ((depth>_zNear) && (depth<_zFar)); }    
    Scalar FPS() const { return _fps; }
    const Matrix3& inv_projection_matrix() const{ return iproj; }
/// @}
    
public:
    /// View+Projection matrix (part of MVP) that considers the kinect sensor intrinsics
    Matrix44f view_matrix(){ return Matrix44f::Identity(); }
    Matrix44f view_projection_matrix();
    Matrix_2x3 projection_jacobian(const Vector3& p);

public:
    Vector2 world_to_image(const Vector3& wrld);
    Vector3 depth_to_world(Real i, Real j, Real depth);
    Vector3 unproject(int i, int j, Scalar depth);
    Vector3 pixel_to_image_plane(int i, int j);
};
