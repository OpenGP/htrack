#include "PoseSpace.h"
#include "util/opencv_wrapper.h"
#include "util/mylogger.h"
#include "util/qfile_helper.h"
#include <QString>
#include "tracker/TwSettings.h"
#include "tracker/Types.h"

bool explore_mode = false;
bool random_pose = false;

#include <fstream>
#include <string>
namespace Eigen{
    template<class Matrix>
    void write_binary(std::string filename, const Matrix& matrix){
        std::ofstream out(filename,ios::out | ios::binary | ios::trunc);
        typename Matrix::Index rows=matrix.rows(), cols=matrix.cols();
        out.write((char*) (&rows), sizeof(typename Matrix::Index));
        out.write((char*) (&cols), sizeof(typename Matrix::Index));
        out.write((char*) matrix.data(), rows*cols*sizeof(typename Matrix::Scalar) );
        out.close();
    }
    template<class Matrix>
    void read_binary(std::string filename, Matrix& matrix){
        std::ifstream in(filename,ios::in | std::ios::binary);
        typename Matrix::Index rows = 0, cols = 0;
        in.read((char*) (&rows),sizeof(typename Matrix::Index));
        in.read((char*) (&cols),sizeof(typename Matrix::Index));
        matrix.resize(rows, cols);
        in.read( (char *) matrix.data() , rows*cols*sizeof(typename Matrix::Scalar) );
        in.close();
    }

    template<class Vector>
    void read_binary_vector(std::string filename, Vector& vector){
        std::ifstream in(filename,ios::in | std::ios::binary);
        typename Vector::Index length = 0;
        in.read((char*) (&length),sizeof(typename Vector::Index));
        vector.resize(length, 1);
        in.read( (char *) vector.data() , length*sizeof(typename Vector::Scalar) );
        in.close();
    }
}

namespace energy{


void PoseSpace::load_pca_data(){
    this->path_pca = local_file_path(subfolder_pca);
    std::cout << "Loading PCA data from: " << path_pca << std::endl;

    Eigen::read_binary_vector<VectorN>(path_pca + "mu", mu);

    if (settings->enable_split_pca) {
        m1 = min(num_thetas_thumb, settings->latent_size);
        m4 = min(num_thetas_fingers, settings->latent_size);
        m = m1 + m4;
        string path_pca_thumb = path_pca + "thumb/";
        Eigen::read_binary<Matrix_MxN>(path_pca_thumb + "P", P1);
        Eigen::read_binary<Matrix_MxN>(path_pca_thumb + "Sigma", Sigma1);
        Eigen::read_binary<Matrix_MxN>(path_pca_thumb + "Limits", Limits1);
        Matrix_MxN P1_block = P1.block(0, 0, n1, m1);  P1 = P1_block;

        Matrix_MxN Sigma1_block = Sigma1.block(0, 0, m1, m1); Sigma1 = Sigma1_block;
        invSigma1 = Sigma1.inverse();

        string path_pca_fingers = path_pca + "fingers/";
        Eigen::read_binary<Matrix_MxN>(path_pca_fingers + "P", P4);
        Eigen::read_binary<Matrix_MxN>(path_pca_fingers + "Sigma", Sigma4);
        Eigen::read_binary<Matrix_MxN>(path_pca_fingers + "Limits", Limits4);
        Matrix_MxN P4_block = P4.block(0, 0, n4, m4);  P4 = P4_block;
        Matrix_MxN Sigma4_block = Sigma4.block(0, 0, m4, m4); Sigma4 = Sigma4_block;
        invSigma4 = Sigma4.inverse();
    }
    else if (settings->enable_joint_pca) {
        m = min(num_thetas_pose, settings->latent_size);
        Eigen::read_binary<Matrix_MxN>(path_pca + "P", P);
        Eigen::read_binary<Matrix_MxN>(path_pca + "Sigma", Sigma);
        Eigen::read_binary<Matrix_MxN>(path_pca + "Limits", Limits);
        Matrix_MxN P_block = P.block(0, 0, n, m);  P = P_block;
        Matrix_MxN Sigma_block = Sigma.block(0, 0, m, m); Sigma = Sigma_block;
        invSigma = Sigma.inverse();
    }
}

void PoseSpace::init(){

    tw_settings->tw_add(settings->enable_split_pca,"Enable","group=PoseSpace");
    tw_settings->tw_add(settings->latent_size,"#dims","group=PoseSpace");
    tw_settings->tw_add(settings->weight_proj,"weight(proj)","group=PoseSpace");
    tw_settings->tw_add(settings->weight_mean,"weight(mean)","group=PoseSpace");

    load_pca_data();
}


void PoseSpace::find_pixel_coordinates_pca(int rows, int cols, const VectorN & x, const Matrix_MxN & Limits, float & pixels_x, float & pixels_y) {
    float axis_start_x = Limits(0, 0);
    float axis_start_y = Limits(1, 0);
    float axis_end_x = Limits(0, 1);
    float axis_end_y = Limits(1, 1);

    float ratio_x = (axis_end_x - axis_start_x) / cols;
    float ratio_y = (axis_end_y - axis_start_y) / rows;

    pixels_x = (x(0) - axis_start_x) / ratio_x;
    pixels_y = rows - (x(1) - axis_start_y) / ratio_y;
}


void PoseSpace::draw_latent_positions_pca(const VectorN & x, const Matrix_MxN & Limits, string path, string window_name, std::vector<VectorN> & x_history) {

    if (!settings->debug_display_latent_space) return;

    cv::Mat dataset_image = cv::imread(path + "pca.png");
    cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);

    float pixels_x, pixels_y;
    find_pixel_coordinates_pca(dataset_image.rows, dataset_image.cols, x, Limits, pixels_x, pixels_y);
    cv::circle(dataset_image, cv::Point(pixels_x, pixels_y), 4.0, cv::Scalar(0, 0, 0), -1, 16);

    x_history.push_back(Vector2(pixels_x, pixels_y));

    for (int i = 0; i < x_history.size() - 1; ++i) {
        cv::line(dataset_image, cv::Point(x_history[i][0], x_history[i][1]), cv::Point(x_history[i + 1][0], x_history[i + 1][1]), cv::Scalar(255, 255, 255), 10);
    }
    cv::circle(dataset_image, cv::Point(x_history[x_history.size() - 1][0], x_history[x_history.size() - 1][1]), 10.0, cv::Scalar(255, 255, 255), -1, 16);


    // Display norm
    /*Scalar difference_norm = (y_real - y_proj).norm();
    char text[255];
    //sprintf(text, "iteration = %d", worker->iteration);
    //cv::putText(dataset_image, text, cv::Point(15, 30), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.2, cv::Scalar(255, 255, 255), 1, CV_AA);
    sprintf(text, "||y - y_proj|| = %7.4f", difference_norm);
    cv::putText(dataset_image, text, cv::Point(15, 55), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.2, cv::Scalar(255, 255, 255), 1, CV_AA);*/

    cv::imshow(window_name, dataset_image);
}


void PoseSpace::compute_linear_approximation(int n, int m, const VectorN & y, const VectorN & x, const Matrix_MxN & P, Matrix_MxN & LHS_E1, VectorN & rhs_E1) {

    Matrix_MxN LHS_e1 = Matrix_MxN::Zero(n, n + m);
    LHS_e1.block(0, 0, n, n) = Matrix_MxN::Identity(n, n);
    LHS_e1.block(0, n, n, m) = -P;
    VectorN rhs_e1 = y - P * x;
    rhs_e1 = - rhs_e1;
    LHS_E1 = 2 * (LHS_e1.transpose() * LHS_e1);
    rhs_E1 = 2 * LHS_e1.transpose() * rhs_e1;
}


void PoseSpace::compute_objective(int n, int m, const VectorN & y, const VectorN & x, const Matrix_MxN & P, const Matrix_MxN & invSigma,
        Matrix_MxN & LHS_E1_y, Matrix_MxN & LHS_E1_x, VectorN & rhs_E1_y, VectorN & rhs_E1_x,
        Matrix_MxN & LHS_E2_y, Matrix_MxN & LHS_E2_x, VectorN & rhs_E2_y, VectorN & rhs_E2_x,
        Matrix_MxN & LHS_E3_y, Matrix_MxN & LHS_E3_x, VectorN & rhs_E3_y, VectorN & rhs_E3_x) {

    VectorN delta_y = VectorN::Zero(n);
    VectorN delta_x = VectorN::Zero(m);

    LHS_E1_y = 2 * Matrix_MxN::Identity(n, n);
    rhs_E1_y = 2 * y.transpose() - 2 * x.transpose() * P.transpose() - 2 * delta_x.transpose() * P.transpose();
    LHS_E1_y = LHS_E1_y.transpose().eval();
    rhs_E1_y = -rhs_E1_y.transpose().eval();

    LHS_E1_x = 2 * (P.transpose() * P);
    rhs_E1_x = 2 * x.transpose() * (P.transpose() * P) - 2 * y.transpose() * P - 2 * delta_y.transpose() * P;
    LHS_E1_x = LHS_E1_x.transpose().eval();
    rhs_E1_x = -rhs_E1_x.transpose().eval();

    LHS_E2_y = 2 * Matrix_MxN::Identity(n, n);
    rhs_E2_y = VectorN::Zero(n);
    LHS_E2_y = LHS_E2_y.transpose().eval();
    rhs_E2_y = -rhs_E2_y.transpose().eval();

    LHS_E2_x = 2 * Matrix_MxN::Identity(m, m);
    rhs_E2_x = VectorN::Zero(m);
    LHS_E2_x = LHS_E2_x.transpose().eval();
    rhs_E2_x = -rhs_E2_x.transpose().eval();

    LHS_E3_y = Matrix_MxN::Zero(n, n);
    rhs_E3_y = VectorN::Zero(n);
    LHS_E3_y = LHS_E3_y.transpose().eval();
    rhs_E3_y = -rhs_E3_y.transpose().eval();

    LHS_E3_x = 2 * invSigma;
    rhs_E3_x = 2 * x.transpose() * invSigma;
    LHS_E3_x = LHS_E3_x.transpose().eval();
    rhs_E3_x = -rhs_E3_x.transpose().eval();


}


void PoseSpace::assemble_joint_system(int n, int m, const Matrix_MxN & LHS_y, const Matrix_MxN & LHS_x,
    const VectorN & rhs_y, const VectorN & rhs_x, Matrix_MxN & LHS, VectorN & rhs) {

    LHS = Matrix_MxN::Zero(n + m, n + m);
    LHS.block(0, 0, n, n) = LHS_y;
    LHS.block(n, n, m, m) = LHS_x;
    rhs = VectorN::Zero(n + m);
    rhs.segment(0, n) = rhs_y;
    rhs.segment(n, m) = rhs_x;

}


void PoseSpace::compose_system(int n, int m, Scalar alpha, Scalar beta, const VectorN & y, const VectorN & x,
    const Matrix_MxN & P, const Matrix_MxN & invSigma, Matrix_MxN & LHS, VectorN & rhs) {
    Matrix_MxN LHS_E1_y, LHS_E1_x, LHS_E2_y, LHS_E2_x, LHS_E3_y, LHS_E3_x;
    VectorN rhs_E1_y, rhs_E1_x, rhs_E2_y, rhs_E2_x, rhs_E3_y, rhs_E3_x;
    compute_objective(n, m, y, x, P, invSigma, LHS_E1_y, LHS_E1_x, rhs_E1_y, rhs_E1_x,
        LHS_E2_y, LHS_E2_x, rhs_E2_y, rhs_E2_x, LHS_E3_y, LHS_E3_x, rhs_E3_y, rhs_E3_x);

    Matrix_MxN LHS_E1, LHS_E2, LHS_E3;
    VectorN rhs_E1, rhs_E2, rhs_E3;

    compute_linear_approximation(n, m, y, x, P, LHS_E1, rhs_E1);
    assemble_joint_system(n, m, LHS_E2_y, LHS_E2_x, rhs_E2_y, rhs_E2_x, LHS_E2, rhs_E2);
    assemble_joint_system(n, m, LHS_E3_y, LHS_E3_x, rhs_E3_y, rhs_E3_x, LHS_E3, rhs_E3);

    LHS = LHS_E1 + alpha * LHS_E2 + beta * LHS_E3;
    rhs = rhs_E1 + alpha * rhs_E2 + beta * rhs_E3;
}

void PoseSpace::track(LinearSystem & system, std::vector<float> _theta){

    if(!(settings->enable_joint_pca || settings->enable_split_pca))
        return;
    Eigen::Map<Thetas> theta(_theta.data());

    if (explore_mode || random_pose) {
        x_history.clear();
        x1_history.clear();
        x4_history.clear();
        explore_mode = false;
        random_pose = false;
    }

    if ((settings->enable_joint_pca &&  m != min(num_thetas_pose,settings->latent_size)) ||
        (settings->enable_split_pca && (m1 != min(num_thetas_thumb,settings->latent_size) || (m4 != min(num_thetas_fingers, settings->latent_size)))))
        load_pca_data();

    Scalar weight_fingers = settings->weight_proj;
    Scalar weight_thumb = settings->weight_proj;
    Scalar weight_mean = settings->weight_mean;

    float alpha = 0 / weight_fingers;
    float beta = weight_mean / weight_fingers;

    LinearSystem system_pca(q + m);
    system_pca.lhs.block(0, 0, q, q) = system.lhs;
    system_pca.rhs.segment(0, q) = system.rhs;
    system = system_pca;

    VectorN y = theta.segment(p, n); y = y - mu;

    Matrix_MxN LHS; VectorN rhs;
    if (settings->enable_joint_pca) {
        VectorN x = P.transpose() * y;
        draw_latent_positions_pca(x, Limits, path_pca, "PCA space", x_history);

        compose_system(n, m, alpha, beta, y, x, P, invSigma, LHS, rhs);

        system.lhs.block(p, p, n + m, n + m) += weight_fingers * LHS;
        system.rhs.segment(p, n + m) += weight_fingers * rhs;
    }

    Matrix_MxN LHS1, LHS4; VectorN rhs1, rhs4;
    if (settings->enable_split_pca) {
        VectorN y1 = y.segment(0, n1);
        VectorN y4 = y.segment(n1, n4);
        VectorN x1 = P1.transpose() * y1;
        VectorN x4 = P4.transpose() * y4;

        draw_latent_positions_pca(x1, Limits1, path_pca + "thumb/", "Thumb PCA space", x1_history);
        draw_latent_positions_pca(x4, Limits4, path_pca + "fingers/", "Fingers PCA space", x4_history);

        Matrix_MxN LHS_e1, LHS_E1, LHS_E2, LHS_E3;
        VectorN rhs_e1, rhs_E1, rhs_E2, rhs_E3;
        {
            LHS_e1 = Matrix_MxN::Zero(n, n + m);
            LHS_e1.block(0, 0, n, n) = Matrix_MxN::Identity(n, n);
            LHS_e1.block(n1, n1, n4, n4) = Matrix_MxN::Zero(n4, n4);
            LHS_e1.block(0, n, n1, m1) = -P1;

            rhs_e1 = VectorN::Zero(n);
            rhs_e1.segment(0, n1) = -y1 + P1 * x1;

            LHS_E1 = 2 * (LHS_e1.transpose() * LHS_e1);
            rhs_E1 = 2 * LHS_e1.transpose() * rhs_e1;

            LHS_E2 = Matrix_MxN::Zero(n + m, n + m);
            LHS_E2.block(0, 0, n1, n1) = 2 * Matrix_MxN::Identity(n1, n1);
            LHS_E2.block(n, n, m1, m1) = 2 * Matrix_MxN::Identity(m1, m1);
            rhs_E2 = VectorN::Zero(n + m);

            LHS_E3 = Matrix_MxN::Zero(n + m, n + m);
            LHS_E3.block(n, n, m1, m1) = 2 * invSigma1;

            rhs_E3 = VectorN::Zero(n + m);
            rhs_E3.segment(n, m1) = -2 * x1.transpose() * invSigma1;

            LHS1 = LHS_E1 + alpha * LHS_E2 + beta * LHS_E3;
            rhs1 = rhs_E1 + alpha * rhs_E2 + beta * rhs_E3;
        }

        system.lhs.block(p, p, n + m, n + m) += weight_thumb * LHS1;
        system.rhs.segment(p, n + m) += weight_thumb *  rhs1;

        {

            LHS_e1 = Matrix_MxN::Zero(n, n + m);
            LHS_e1.block(0, 0, n, n) = Matrix_MxN::Identity(n, n);
            LHS_e1.block(0, 0, n1, n1) = Matrix_MxN::Zero(n1, n1);
            LHS_e1.block(n1, n + m1, n4, m4) = -P4;

            rhs_e1 = VectorN::Zero(n);
            rhs_e1.segment(n1, n4) = -y4 + P4 * x4;

            LHS_E1 = 2 * (LHS_e1.transpose() * LHS_e1);
            rhs_E1 = 2 * LHS_e1.transpose() * rhs_e1;

            LHS_E2 = Matrix_MxN::Zero(n + m, n + m);
            LHS_E2.block(n1, n1, n4, n4) = 2 * Matrix_MxN::Identity(n4, n4);
            LHS_E2.block(n + m1, n1 + m1, m4, m4) = 2 * Matrix_MxN::Identity(m4, m4);
            rhs_E2 = VectorN::Zero(n + m);

            LHS_E3 = Matrix_MxN::Zero(n + m, n + m);
            LHS_E3.block(n + m1, n + m1, m4, m4) = 2 * invSigma4;

            rhs_E3 = VectorN::Zero(n + m);
            rhs_E3.segment(n + m1, m4) = -2 * x4.transpose() * invSigma4;

            LHS4 = LHS_E1 + alpha * LHS_E2 + beta * LHS_E3;
            rhs4 = rhs_E1 + alpha * rhs_E2 + beta * rhs_E3;
        }

        system.lhs.block(p, p, n + m, n + m) += weight_fingers * LHS4;
        system.rhs.segment(p, n + m) += weight_fingers * rhs4;
    }
}

} ///< energy::
