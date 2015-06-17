#include "Camera.h"

// Vector2i Camera::COL2DEPTHOFFSET;

//Camera::Matrix44f Camera::view_matrix(){
//    // static Matrix44f view = Eigen::lookAt(Vector3(0,0,0), Vector3(0,0,1), Vector3(0,1,0));
//    ///--- This one creates a ****left-handed**** coordinate system
//    static Matrix44f view = Matrix44f::Identity();
//    // view(2,2) = -1;
//    return view;
//}

Camera::Camera(CAMERAMODE mode, int fps):_mode(mode), _fps(fps){
    /// By default we only care from 30cm to 1.5m
    _zNear = 300 /*mm*/;
    _zFar = 1500 /*mm*/;

    switch(_mode){
        case VGA:
            _width = 640;
            _height = 480;
            _focal_length_x = 525.5;
            _focal_length_y = 525.5;
            break;
        case QVGA:
            _width = 320;
            _height = 240;
            _focal_length_x = 287.26;
            _focal_length_y = 287.26;
            //_zFar = 800;
            _zFar = 1000;
            break;
        case Tompson:
            _width = 640;
            _height = 480;
#ifdef FIX_CALIB_PARS
            // _focal_length_x = 557.5;
            //_focal_length_y = 557.5;
#else
            _focal_length_x = 588.036865;
            _focal_length_y = 587.075073;
#endif
            break;
        case Tang: ///< same camera as Chen?
        case Chen:
            _width = 320;
            _height = 240;
            _focal_length_x = 241.42;
            _focal_length_y = 241.42;
            _zNear = 150;
            _zFar = 600;
        case Intel:
           _width = 320;
           _height = 240;
           _focal_length_x = 224.502;
           _focal_length_y = 230.494;
           _zFar = 500;
           _zNear = 100;
            break;
        default:
            printf("!!!FATAL: Invalid Camera");
            exit(0);
    }

    ///--- Assemble projection matrix
    auto kinectproj = [=]() {
        Matrix33f cam_matrix = Matrix33f::Zero();
        cam_matrix (0, 0) = _focal_length_x; /// FocalLength X
        cam_matrix (1, 1) = _focal_length_y; /// FocalLength Y
        cam_matrix (0, 2) = _width / 2;      /// CameraCenter X
        cam_matrix (1, 2) = _height / 2;     /// CameraCenter Y
        cam_matrix (2, 2) = 1.0;
        return cam_matrix;
    };
    proj = kinectproj();
    iproj = proj.inverse();
    // cout << "Camera::proj" << endl << proj  << endl;
    // cout << "Camera::iproj" << endl << iproj << endl;
}

Vector3 Camera::depth_to_world(Real i, Real j, Real depth){
#ifndef USE_TOMPSON_FIX
    Vector3 wrld = iproj * Vector3( i*depth, (height()-j-1)*depth, depth );
#else
    Vector3 wrld = iproj * Vector3( (i + 1)*depth, (height() - (j + 1)) * depth, depth );
#endif
    return wrld;
}

Vector3 Camera::unproject(int i, int j, Scalar depth){
    return pixel_to_image_plane(i,j) * depth;
}

Vector3 Camera::pixel_to_image_plane(int i, int j){
    Scalar x = (i-proj(0,2))/proj(0,0);
    Scalar y = (j-proj(1,2))/proj(1,1);
    return Vector3(x,y,1);
}

Vector2 Camera::world_to_image(const Vector3& wrld){
    Scalar x = wrld[0]/wrld[2];
    Scalar y = wrld[1]/wrld[2];
    x = x*proj(0,0) + proj(0,2);
    y = y*proj(1,1) + proj(1,2);
    return Vector2(x,y);
}

Camera::Matrix44f Camera::view_projection_matrix(){
    ///--- Intrinsics matrix
    Matrix33f& K = proj;
    int w = this->width();
    int h = this->height();

    Matrix44f mat = Matrix44f::Identity();
    mat(0,0) = 2.0/(float)w*K(0,0); // use camera instrinsics and convert to GL [0,h] => [-1,1]
    mat(0,2) = (2.0/(float)w*(K(0,2)+0.5))-1.0; // 0.5 offset as GL pixel middle point is at 0.5,0.5
    // Y
    mat(1,1) = 2.0/(float)h*K(1,1); // use camera instrinsics and convert to GL [0,h] => [-1,1]
    mat(1,2) = (2.0/(float)h*(K(1,2)+0.5))-1.0;
    // Z
    mat(2,2) = (_zFar+_zNear)/(_zFar-_zNear);
    mat(2,3) = -2.0*_zFar*_zNear/(_zFar-_zNear);
    // W
    mat(3,2) = 1; // not as in GL where it would be -1
    mat(3,3) = 0;

    return mat;
}

Matrix_2x3 Camera::projection_jacobian(const Vector3 &p){
    Matrix_2x3 M;
    M << _focal_length_x/p.z(), 0, -p.x() * _focal_length_x/(p.z()*p.z()),
         0, _focal_length_y/p.z(),  -p.y() * _focal_length_y/(p.z()*p.z());
    return M;
}

