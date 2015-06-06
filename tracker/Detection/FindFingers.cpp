#include "FindFingers.h"

#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <bitset>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/eigen.hpp>

#include <iostream>
#include <fstream>
#include <iomanip>
#include <stdlib.h>
#include <stdio.h>
#include <algorithm>
#include <set>
#include <limits>

#include "tracker/HandFinder/HandFinder.h"
#include "tracker/DataStructure/SkeletonSerializer.h"
#include "DetectionStream.h"
#include "matlab_helpers.h"

FindFingers::FindFingers(Worker *worker, DetectionStream *detection) : worker(worker), detection(detection), camera(worker->camera) {}

inline bool float_comparator(std::pair<Scalar, size_t> a, std::pair<Scalar, size_t> b) {
    return a.first < b.first;
}

void FindFingers::eigen_imshow(Matrix_MxN eigen_image, string window_name) {
    cv::Mat cv_image; eigen2cv(eigen_image, cv_image);
    cv::normalize(cv_image, cv_image, 0, 1, cv::NORM_MINMAX);
    cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);
    cv::imshow(window_name, cv_image);
}

void FindFingers::print_queue(const Matrix_MxN &depth, std::set<std::pair<float, size_t> > queue) {
    cout << "size = " << queue.size() << endl;
    for (auto it = queue.begin(); it != queue.end(); it++) {
        Vector2s v = matlab::ind2sub(M, N, it->second);
        cout << "d = " << it->first << ", (" << v(0) + 1 << ", " << v(1) + 1 << ")" << endl;
    }
    cout << endl;
}

Matrix_MxN FindFingers::crop_image(const Matrix_MxN &depth) {
    if (call_stack) cout << "[FindFingers::crop_image()]" << endl;

    // Clean the arrays
    thumb_index = -1;
    hand_side = -1;
    if (xy_finger_segments.size()) xy_finger_segments.clear();
    if (z_finger_segments.size()) z_finger_segments.clear();
    if (xy_finger_tips.size()) xy_finger_tips.clear();
    if (z_finger_tips.size()) z_finger_tips.clear();
    if (xy_finger_roots.size()) xy_finger_roots.clear();
    if (z_finger_roots.size()) z_finger_roots.clear();

    // Crop the image
    min_column = numeric_limits<size_t>::max();
    max_column = 0;
    min_row = numeric_limits<size_t>::max();
    max_row = 0;
    for (size_t i = 0; i < depth.rows(); i++) {
        for (size_t j = 0; j < depth.cols(); j++) {
            if (depth(i, j) > 0) {
                if (i < min_row) min_row = i;
                if (i > max_row) max_row = i;
                if (j < min_column) min_column = j;
                if (j > max_column) max_column = j;
            }
        }
    }
    if (min_row > 0) min_row--;
    if (max_row < depth.rows() - 1) max_row++;
    if (min_column > 0) min_column--;
    if (max_column < depth.cols() - 1) max_column++;
    M = max_row - min_row + 1;
    N = max_column - min_column + 1;
    Matrix_MxN cropped_depth = Matrix_MxN::Zero(M, N);
    for (size_t i = 0; i < M; i++) {
        for (size_t j = 0; j < N; j++) {
            cropped_depth(i, j) = depth(i + min_row, j + min_column);
        }
    }
    return cropped_depth;
}

void FindFingers::restore_original_image() {
    if (call_stack) cout << "[FindFingers::restore_original_image()]" << endl;
    if (downsampled) {
        for (size_t p = 0; p < xy_finger_tips.size(); p++) {
            xy_finger_tips[p](0) *= 2;
            xy_finger_tips[p](1) *= 2;
            xy_finger_roots[p](0) *= 2;
            xy_finger_roots[p](1) *= 2;
        }
        for (size_t p = 0; p < z_finger_tips.size(); p++) {
            z_finger_tips[p](0) *= 2;
            z_finger_tips[p](1) *= 2;
            z_finger_roots[p](0) *= 2;
            z_finger_roots[p](1) *= 2;
        }
        palm_center(0) *= 2;
        palm_center(1) *= 2;
    }

    for (size_t p = 0; p < xy_finger_tips.size(); p++) {
        xy_finger_tips[p](0) += min_row;
        xy_finger_tips[p](1) += min_column;
        xy_finger_roots[p](0) += min_row;
        xy_finger_roots[p](1) += min_column;
    }
    for (size_t p = 0; p < z_finger_tips.size(); p++) {
        z_finger_tips[p](0) += min_row;
        z_finger_tips[p](1) += min_column;
        z_finger_roots[p](0) += min_row;
        z_finger_roots[p](1) += min_column;
    }

    palm_center(0) += min_row;
    palm_center(1) += min_column;

}

Matrix_MxN FindFingers::downsample_image(const Matrix_MxN &depth) {
    if (call_stack) cout << "[FindFingers::downsample_image()]" << endl;

    downsampled = true;

    model_finger_length = 30;
    min_number_of_pixels_in_finger = 11;
    local_min_window = 4;
    model_phalangue_length = 14;
    finger_radius_threshold = 5;
    min_finger_segment_length = 8;

    size_t M = depth.rows() / 2;
    size_t N = depth.cols() / 2;

    Matrix_MxN downsampled_depth = Matrix_MxN::Zero(M, N);
    for (size_t i = 0; i < M; i++) {
        for (size_t j = 0; j < N; j++) {
            downsampled_depth(i, j) = depth(i * 2, j * 2);
        }
    }
    return downsampled_depth;
}

void FindFingers::display_all_fingers(const Matrix_MxN &depth) {
    if (call_stack) cout << "[FindFingers::display_all_fingers()]" << endl;

    size_t scale = 3;
    size_t u, v;

    std::vector<cv::Scalar> colors;
    colors.push_back(cv::Scalar(0.4, 0.0, 0.9));
    colors.push_back(cv::Scalar(0.0, 0.5, 1.0));
    colors.push_back(cv::Scalar(0.0, 0.8, 1.0));
    colors.push_back(cv::Scalar(0.3, 0.8, 0.0));
    colors.push_back(cv::Scalar(1.0, 0.6, 0.0));
    size_t count = 0;

    cv::Mat image = cv::Mat::zeros(depth.rows(), depth.cols(), CV_32FC3);

    for (size_t i = min_row; i <= max_row; i++) {
        for (size_t j = min_column; j <= max_column; j++) {
            if (depth(i, j) > 0) image.at<cv::Vec3f>(i, j) = cv::Vec3f(1.0, 1.0, 1.0);

            if (downsampled) {
                u = (i - min_row) / 2;
                v = (j - min_column) / 2;
                if (u >= M || v >= N) continue;
                for (size_t p = 0; p < xy_finger_segments.size(); p++)
                    if (xy_finger_segments[p](u, v) > 0) image.at<cv::Vec3f>(i, j) = cv::Vec3f(0.9, 0.9, 0.85);
                for (size_t p = 0; p < z_finger_segments.size(); p++)
                    if (z_finger_segments[p](u, v) > 0) image.at<cv::Vec3f>(i, j) = cv::Vec3f(0.7, 0.7, 0.6);
            }

            else {
                for (size_t p = 0; p < xy_finger_segments.size(); p++)
                    if (xy_finger_segments[p](i - min_row, j - min_column) > 0) image.at<cv::Vec3f>(i, j) = cv::Vec3f(0.9, 0.9, 0.85);
                for (size_t p = 0; p < z_finger_segments.size(); p++)
                    if (z_finger_segments[p](i - min_row, j - min_column) > 0) image.at<cv::Vec3f>(i, j) = cv::Vec3f(0.7, 0.7, 0.6);

            }
        }
    }
    cv::resize(image, image, cv::Size(depth.cols() * scale, depth.rows() * scale));
    for (size_t p = 0; p < xy_finger_tips.size(); p++) {
        cv::Scalar color = colors[count]; if (count == thumb_index) color = cv::Scalar(0.7, 0.0, 0.5);
        //cv::line(image, cv::Point(scale * xy_finger_roots[p](1), scale * xy_finger_roots[p](0)), cv::Point(scale * palm_center(1), scale * palm_center(0)), cv::Scalar(1.0, 1.0, 0.855), 2.5, CV_AA);
        cv::circle(image, cv::Point(scale * xy_finger_tips[p](1), scale * xy_finger_tips[p](0)), 7.0, color, -1, 16);
                   cv::line(image, cv::Point(scale * xy_finger_tips[p](1), scale * xy_finger_tips[p](0)),
                                             cv::Point(scale * xy_finger_roots[p](1), scale * xy_finger_roots[p](0)), color, 5);
                                             count++;
    }
    for (size_t p = 0; p < z_finger_tips.size(); p++) {
        cv::Scalar color = colors[count]; if (count == thumb_index) color = cv::Scalar(0.7, 0.0, 0.5);
        //cv::line(image, cv::Point(scale * z_finger_roots[p](1), scale * z_finger_roots[p](0)), cv::Point(scale * palm_center(1), scale * palm_center(0)), cv::Scalar(1.0, 1.0, 0.85), 2.5, CV_AA);
        cv::circle(image, cv::Point(scale * z_finger_tips[p](1), scale * z_finger_tips[p](0)), 7.0, color, -1, 16);
                   cv::line(image, cv::Point(scale * z_finger_tips[p](1), scale * z_finger_tips[p](0)),
                                             cv::Point(scale * z_finger_roots[p](1), scale * z_finger_roots[p](0)), color, 5);
                                             count++;
    }

    cv::circle(image, cv::Point(scale * palm_center(1), scale * palm_center(0)), 10.0, cv::Scalar(0.0, 0.0, 1.0), -1, 16);
    /*cv::circle(image, cv::Point(scale * wristband_center(1), scale * wristband_center(0)), 5.0, cv::Scalar(1.0, 0.0, 0.0), -1, 16);
        cv::line(image, cv::Point(scale * wristband_center(1), scale * wristband_center(0)),
        cv::Point(scale * (wristband_center(1) + 50 * wristband_direction(1)),
        scale * (wristband_center(0) + 50 * wristband_direction(0))), cv::Scalar(1.0, 0.2, 0.2), 4, CV_AA);
        cv::line(image, cv::Point(scale * wristband_center(1), scale * wristband_center(0)),
        cv::Point(scale * thumb_tip(1), scale * thumb_tip(0)), cv::Scalar(1.0, 0.2, 0.2), 4, CV_AA);*/
    cv::resize(image, image, cv::Size(image.cols / 2, image.rows / 2));
    cv::imshow("FindFingers", image);
}

void FindFingers::get_neighbors(const Matrix_MxN &depth, const Matrix_MxN_b &mask, Matrix_MxN &distance, Matrix_MxN &visited, std::set<std::pair<float, size_t> > &queue, Vector2s v0, FindFingers::Mode mode) {

    int t = 1;
    size_t r0 = v0(0);
    size_t c0 = v0(1);
    for (int u = -t; u <= t; u++) {
        for (int w = -t; w <= t; w++) {

            int r = r0 + u;
            int c = c0 + w;
            if (u == 0 && w == 0) continue;
            if (r < 0 || r >= mask.rows() || c < 0 || c >= mask.cols()) continue;

            if (mask(r, c) == 1 && visited(r, c) == 0) {
                Vector2s v = Vector2s(r, c);

                float d;
                if (mode == XY) {
                    Vector2 difference = Vector2((float)v(0) - (float)v0(0), (float)v(1) - (float)v0(1));
                    d = difference.norm();
                }
                else if (mode == Z) {
                    d = abs(depth(r0, c0) - depth(r, c));
                }

                if (distance(r, c) == -1 || distance(r, c) > distance(r0, c0) + d) {
                    distance(r, c) = distance(r0, c0) + d;
                    size_t i = matlab::sub2ind(M, N, v);
                    float d = distance(v(0), v(1));
                    queue.insert(std::pair<float, size_t>(d, i));
                    visited(r, c) = 1;
                }
            }
        }
    }
}

void FindFingers::dijkstra(const Matrix_MxN &depth, Matrix_MxN &distance, Vector2s v0, float threshold, FindFingers::Mode mode) {
    if (call_stack) cout << "[FindFingers::dijkstra()]" << endl;

    Matrix_MxN_b mask = depth.array() > 0;
    std::set<std::pair<float, size_t>> queue;
    std::vector<Vector2s> neighbors;
    Matrix_MxN visited = Matrix_MxN::Zero(M, N);
    distance(v0(0), v0(1)) = 0;
    get_neighbors(depth, mask, distance, visited, queue, v0, mode);

    while (!queue.empty()) {
        auto it = queue.begin();
        size_t i = it->second;
        queue.erase(it);
        Vector2s v = matlab::ind2sub(M, N, i);
        get_neighbors(depth, mask, distance, visited, queue, v, mode);
        if (distance(v(0), v(1)) > threshold) break;
    }
    //distance(v0(0), v0(1)) = 1;
}

void FindFingers::get_neighbors_vector(const Matrix_MxN &depth, const Matrix_MxN_b &mask, Matrix_MxN &distance, Matrix_MxN &visited, std::vector<std::pair<float, size_t> > &queue, Vector2s v0, FindFingers::Mode mode) {
    int t = 1;
    size_t r0 = v0(0);
    size_t c0 = v0(1);
    for (int u = -t; u <= t; u++) {
        for (int w = -t; w <= t; w++) {

            int r = r0 + u;
            int c = c0 + w;
            if (u == 0 && w == 0) continue;
            if (r < 0 || r >= mask.rows() || c < 0 || c >= mask.cols()) continue;

            if (mask(r, c) == 1 && visited(r, c) == 0) {
                Vector2s v = Vector2s(r, c);

                float d;
                if (mode == XY) {
                    Vector2 difference = Vector2((float)v(0) - (float)v0(0), (float)v(1) - (float)v0(1));
                    d = difference.norm();
                }
                else if (mode == Z) {
                    d = abs(depth(r0, c0) - depth(r, c));
                }

                if (distance(r, c) == -1 || distance(r, c) > distance(r0, c0) + d) {
                    distance(r, c) = distance(r0, c0) + d;
                    size_t i = matlab::sub2ind(M, N, v);
                    float d = distance(v(0), v(1));
                    queue.push_back(std::pair<float, size_t>(d, i));
                    visited(r, c) = 1;
                }
            }
        }
    }
}

void FindFingers::dijkstra_vector(const Matrix_MxN &depth, Matrix_MxN &distance, Vector2s v0, float threshold, FindFingers::Mode mode) {
    if (call_stack) cout << "[FindFingers::dijkstra_vector()]" << endl;

    Matrix_MxN_b mask = depth.array() > 0;
    std::vector<std::pair<float, size_t>> queue;
    std::vector<Vector2s> neighbors;
    Matrix_MxN visited = Matrix_MxN::Zero(M, N);
    distance(v0(0), v0(1)) = 0;
    get_neighbors_vector(depth, mask, distance, visited, queue, v0, mode);

    while (!queue.empty()) {

        float min_distance = numeric_limits<float>::max();
        size_t min_index = -1;
        for (size_t k = 0; k < queue.size(); k++) {
            if (queue[k].first < min_distance) {
                min_distance = queue[k].first;
                min_index = k;
            }
        }
        size_t i = queue[min_index].second;
        queue[min_index] = queue[queue.size() - 1];
        queue.pop_back();

        Vector2s v = matlab::ind2sub(M, N, i);
        get_neighbors_vector(depth, mask, distance, visited, queue, v, mode);
        if (distance(v(0), v(1)) > threshold) break;
    }
    //distance(v0(0), v0(1)) = 1;
}

void FindFingers::crop_finger(const Matrix_MxN &depth, Matrix_MxN &finger) {
    if (call_stack) cout << "[FindFingers::crop_finger()]" << endl;

    float max_distance = finger.maxCoeff();
    float bin_size = max_distance / number_of_hist_bins;
    VectorN bins_limits = VectorN::Zero(number_of_hist_bins + 1);
    for (size_t i = 1; i < number_of_hist_bins + 1; i++) {
        bins_limits(i) = bin_size * i;
    }

    std::vector<std::vector<size_t>> bins_indices = std::vector<std::vector<size_t>>(number_of_hist_bins, std::vector<size_t>());
    VectorN bins_counts = VectorN::Zero(number_of_hist_bins);
    for (size_t u = 0; u < finger.rows(); u++) {
        for (size_t v = 0; v < finger.cols(); v++) {
            if (finger(u, v) < 0) continue;
            for (size_t w = 0; w < number_of_hist_bins; w++) {
                if (finger(u, v) > bins_limits(w) && finger(u, v) <= bins_limits(w + 1)) {
                    size_t i = matlab::sub2ind(finger.rows(), finger.rows(), Vector2s(u, v));
                    bins_indices[w].push_back(i);
                }
            }
        }
    }
    for (size_t i = 0; i < number_of_hist_bins; i++) {
        bins_counts(i) = bins_indices[i].size();
    }

    float mean_counts = bins_counts.segment(start_histogram_mean, end_histogram_mean - start_histogram_mean).mean();
    float std_counts = matlab::stddev(bins_counts.segment(start_histogram_mean, end_histogram_mean - start_histogram_mean));

    size_t last_inlier_index = number_of_hist_bins - 1;
    for (size_t i = 0; i < number_of_hist_bins; i++) {
        if (bins_counts(i) < mean_counts + 3 * std_counts) last_inlier_index = i;
    }
    for (size_t u = last_inlier_index + 1; u < number_of_hist_bins; u++) {
        for (size_t v = 0; v < bins_indices[u].size(); v++) {
            size_t i = bins_indices[u][v];
            Vector2s s = matlab::ind2sub(M, N, i);
            finger(s(0), s(1)) = -1;
        }
    }
}

std::set<std::pair<float, size_t> > FindFingers::find_local_min(const Matrix_MxN &depth) {
    if (call_stack) cout << "[FindFingers::find_local_min()]" << endl;

    int offset = local_min_window;
    Matrix_MxN min_store = Matrix_MxN::Zero(M, N);

    for (int r = 0; r < M; r++) {
        for (int c = 0; c < N; c++) {
            if (depth(r, c) == 0) continue;

            float min_value = numeric_limits<float>::max();
            for (int u = -offset; u <= offset; u++) {
                if (r - u < 0 || r + u >= M) continue;
                if (depth(r + u, c) > 0 && depth(r + u, c) < min_value) {
                    min_value = depth(r + u, c);
                    min_store(r, c) = min_value;
                }
            }
        }
    }
    Matrix_MxN result = Matrix_MxN::Zero(M, N);
    for (int r = 0; r < M; r++) {
        for (int c = 0; c < N; c++) {
            if (depth(r, c) == 0) continue;
            float min_value = numeric_limits<float>::max();
            for (int v = -offset; v <= offset; v++) {
                if (c - v < 0 || c + v >= N) continue;
                if (min_store(r, c + v) > 0 && min_store(r, c + v) < min_value)
                    min_value = min_store(r, c + v);
            }
            if (depth(r, c) == min_value) {
                result(r, c) = 1;
            }
        }
    }
    for (int r = M - 1; r >= 0; r--) {
        for (int c = N - 1; c >= 0; c--) {
            if (result(r, c) == 0) continue;
            for (int u = -offset; u <= offset; u++) {
                for (int v = -offset; v <= offset; v++) {
                    if (u == 0 && v == 0) continue;
                    if (r - u < 0 || r + u >= M || c - v < 0 || c + v >= N) continue;
                    result(r + u, c + v) = 0;
                }
            }
        }
    }
    std::set<std::pair<float, size_t>> indices;
    for (size_t r = 0; r < M; r++) {
        for (size_t c = 0; c < N; c++) {
            if (result(r, c) > 0) {
                size_t i = matlab::sub2ind(M, N, Vector2s(r, c));
                indices.insert(std::pair<float, size_t>(depth(r, c), i));
            }
        }
    }
    //eigen_imshow(result); waitKey(0);
    return indices;
}

void FindFingers::compute_finger_pixels_svd(const Matrix_MxN & finger, Matrix_Nx2& data, Eigen::JacobiSVD<Matrix_2x2> & svd,
                                            float & mean_rows, float & mean_columns) {
    if (call_stack) cout << "[FindFingers::compute_finger_pixels_svd()]" << endl;

    VectorNs R, C; matlab::find<Matrix_MxN_b>(finger.array() > -1, R, C);
    size_t n = R.size();
    data = Eigen::Matrix<Scalar, Eigen::Dynamic, 2>::Zero(n, 2);
    mean_rows = matlab::mean(R); mean_columns = matlab::mean(C);
    data.block(0, 0, n, 1) = R.array().cast<float>() - mean_rows;
    data.block(0, 1, n, 1) = C.array().cast<float>() - mean_columns;
    Eigen::Matrix<Scalar, 2, 2> covariance = data.transpose() * data / n;
    svd = Eigen::JacobiSVD<Matrix_2x2>(covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);
    svd.computeU();
    //cout << svd.matrixU() << endl;
}

bool FindFingers::verify_finger_shape(const Matrix_MxN &finger, Vector2s v, FindFingers::Mode mode) {
    if (call_stack) cout << "[FindFingers::verify_finger_shape()]" << endl;

    Eigen::JacobiSVD<Eigen::Matrix<Scalar, 2, 2>> svd;
    Eigen::Matrix<Scalar, Eigen::Dynamic, 2> data;
    float mean_rows, mean_columns;
    compute_finger_pixels_svd(finger, data, svd, mean_rows, mean_columns);
    VectorN s = svd.singularValues().array().pow(0.5);

    if (mode == XY) {
        float ratio = s(0) / s(1);
        if (ratio < finger_ratio_threshold) return false;
        else return true;
    }
    if (mode == Z) {
        // Discard too thick finger candidates
        if (2 * s(1) > finger_radius_threshold) return false;

        // Find center of mass of finger point cloud
        Matrix_MxN U = svd.matrixU();
        Vector2 cloud_center = Vector2(v(0) - mean_rows, v(1) - mean_columns);
        cloud_center = cloud_center.transpose() * U;
        cloud_center(1) = 0.0;

        // Look at points distribution on the [R, 3R] ring around the fingertip
        float cloud_radius = 2 * s(1);
        data = data * U;

        size_t number_of_points_inside_ring = 0;
        size_t number_of_points_inside_tube = 0;
        for (size_t d = 0; d < data.rows(); d++) {
            VectorN point = data.row(d);
            if ((point - cloud_center).norm() >= cloud_radius && (point - cloud_center).norm() < 3 * cloud_radius) {
                number_of_points_inside_ring++;
                if (point(1) >= cloud_center(1) - cloud_radius && point(1) <= cloud_center(1) + cloud_radius) {
                    number_of_points_inside_tube++;
                }
            }
        }
        if ((float)number_of_points_inside_tube / number_of_points_inside_ring < distribution_ratio_threshold)
            return false;
        else return true;
    }
    return false;
}

bool FindFingers::find_xy_fingers(Matrix_MxN &depth) {
    if (call_stack) cout << "[FindFingers::find_xy_fingers()]" << endl;

    Matrix_MxN_b mask = depth.array() > 0;

    Matrix_MxN distance = -1 * Matrix_MxN::Ones(M, N);
    VectorNs R, C; matlab::find<Matrix_MxN_b>(mask, R, C);
    if (R.cols() == 0 || R.rows() == 0 || C.rows() == 0 || C.cols() == 0) return false;

    Vector2s v = Vector2s(round(R.mean()), round(C.mean()));

    for (size_t t = 0; t < num_fingers; t++) {
        // Find geodesic extremum
        dijkstra_vector(depth, distance, v, numeric_limits<float>::max(), XY);
        distance.maxCoeff(&v(0), &v(1));
        distance(v(0), v(1)) = 0;

        // Grow finger segment
        Matrix_MxN finger = -1 * Matrix_MxN::Ones(M, N);
        dijkstra_vector(depth, finger, v, model_finger_length, XY);

        // Crop the segment at its base
        crop_finger(depth, finger);

        // Discard too big or too small fingers
        VectorNs I; matlab::find<Matrix_MxN_b>(finger.array() > -1, I);
        if (I.size() < min_number_of_pixels_in_finger) continue;
        if (I.size() > max_fraction_of_pixels_in_finger * num_of_pixels_in_hand) continue;

        // Discard too short segments
        if (!verify_finger_shape(finger, v, XY)) continue;

        // Remove processed fingers
        mask = mask.array() * (finger.array() == -1);
        distance = distance.array() * mask.array().cast<float>();
        depth = depth.array() * mask.array().cast<float>();

        xy_finger_segments.push_back(finger);
        xy_finger_tips.push_back(v.cast<float>());
    }
    return true;
}

bool FindFingers::find_z_fingers(const Matrix_MxN &xy_depth, Matrix_MxN &z_depth) {
    if (call_stack) cout << "[FindFingers::find_z_fingers()]" << endl;

    std::set<std::pair<float, size_t>> indices = find_local_min(z_depth);

    Matrix_MxN_b mask = z_depth.array() > 0;

    for (auto it = indices.begin(); it != indices.end(); it++) {
        size_t i = it->second;
        Vector2s v = matlab::ind2sub(M, N, i);
        size_t r = v(0); size_t c = v(1);

        // Discard fingers that are already considered
        if (z_depth(r, c) == 0) continue;
        if (xy_depth(r, c) == 0) continue;
        if (xy_finger_segments.size() + z_finger_segments.size() >= num_fingers) break;

        // Grow segment
        Matrix_MxN finger = -1 * Matrix_MxN::Ones(M, N);
        dijkstra_vector(z_depth, finger, v, model_phalangue_length, Z);

        //eigen_imshow(finger); waitKey(0);

        // Discard too big finger candidates
        VectorNs I; matlab::find<Matrix_MxN_b>(finger.array() > -1, I);
        if (I.size() < min_number_of_pixels_in_finger) continue;
        if (I.size() > max_fraction_of_pixels_in_finger * num_of_pixels_in_hand) continue;

        // Discard too short segments
        if (!verify_finger_shape(finger, v, Z)) continue;

        // Remove processed fingers
        mask = mask.array() * (finger.array() == -1);
        z_depth = z_depth.array() * mask.array().cast<float>();

        z_finger_segments.push_back(finger);
        z_finger_tips.push_back(v.cast<float>());
    }
    return true;
}

void FindFingers::find_finger_directions(FindFingers::Mode mode) {
    if (call_stack) cout << "[FindFingers::find_finger_directions()]" << endl;

    size_t num_detected_fingers;
    if (mode == XY) {
        num_detected_fingers = xy_finger_tips.size();
        xy_finger_roots = std::vector<Vector2>(num_detected_fingers, Vector2::Zero(2));
    }
    if (mode == Z) {
        num_detected_fingers = z_finger_tips.size();
        z_finger_roots = std::vector<Vector2>(num_detected_fingers, Vector2::Zero(2));
    }
    for (size_t i = 0; i < num_detected_fingers; i++) {

        Eigen::JacobiSVD<Eigen::Matrix<Scalar, 2, 2>> svd; Eigen::Matrix<Scalar, Eigen::Dynamic, 2> data;
        float mean_rows, mean_columns;
        VectorN v;
        if (mode == XY) {
            compute_finger_pixels_svd(xy_finger_segments[i], data, svd, mean_rows, mean_columns);
            v = xy_finger_tips[i];
        }
        if (mode == Z) {
            compute_finger_pixels_svd(z_finger_segments[i], data, svd, mean_rows, mean_columns);
            v = z_finger_tips[i];
        }
        VectorN s = svd.singularValues().array().pow(0.5);
        Matrix_MxN U = svd.matrixU();

        Vector2 finger_tip, finger_root;
        if (mode == XY) {
            finger_tip = Vector2(v(0) - mean_rows, v(1) - mean_columns);
            finger_tip = finger_tip.transpose() * U;
            finger_tip(1) = 0.0;
            finger_root = Vector2(finger_tip(0) - matlab::sign<float>(finger_tip(0)) * (2 * s(0) + 2 * s(1)), 0);
        }
        if (mode == Z) {
            if (s.maxCoeff() < min_finger_segment_length / 2) {
                finger_tip = Vector2(0.0, 0.0);
                finger_root = Vector2(0.0, 0.0);
            }
            else {
                finger_tip = Vector2(s(0) + s(1), 0.0);
                finger_root = Vector2(-s(0) - s(1), 0.0);
            }
        }
        // Transform back
        Matrix_MxN Uinv = U.inverse();
        finger_tip = finger_tip.transpose() * Uinv;
        finger_root = finger_root.transpose() * Uinv;
        finger_tip = Vector2(finger_tip(0) + mean_rows, finger_tip(1) + mean_columns);
        finger_root = Vector2(finger_root(0) + mean_rows, finger_root(1) + mean_columns);

        if (mode == XY) {
            xy_finger_tips[i] = finger_tip;
            xy_finger_roots[i] = finger_root;
        }
        if (mode == Z) {
            z_finger_tips[i] = finger_tip;
            z_finger_roots[i] = finger_root;
        }
    }
}

void FindFingers::find_palm(const Matrix_MxN &xy_depth, const Matrix_MxN &z_depth) {
    Matrix_MxN_b indicator = z_depth.array() > 0;
    Matrix_MxN depth = xy_depth.array() * indicator.cast<float>().array();
    VectorNs R, C; matlab::find<Matrix_MxN>(depth, R, C);
    VectorN D = VectorN::Zero(R.size());
    for (int i = 0; i < R.size(); ++i) {
        D(i) = depth(R(i), C(i));
    }
    size_t n = R.size();
    Eigen::Matrix<Scalar, Eigen::Dynamic, 3> data = Matrix_MxN::Zero(n, 3);
    float mean_rows = matlab::mean(R);
    float mean_columns = matlab::mean(C);
    float mean_values = matlab::mean(D);
    data.block(0, 0, n, 1) = R.array().cast<float>() - mean_rows;
    data.block(0, 1, n, 1) = C.array().cast<float>() - mean_columns;
    data.block(0, 2, n, 1) = D.array().cast<float>() - mean_values;
    Eigen::Matrix<Scalar, 3, 3> covariance = data.transpose() * data / n;
    Eigen::JacobiSVD<Eigen::Matrix<Scalar, 3, 3>> svd = Eigen::JacobiSVD<Eigen::Matrix<Scalar, 3, 3>>(covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);
    palm_center = Vector3(mean_rows, mean_columns, mean_values);
    Matrix_MxN U = svd.matrixU();
    palm_normal = U.col(2);
    //cout << palm_center(0) + min_row <<" " << palm_center(1) + min_column << " " << palm_center(2) << endl;

}

float FindFingers::get_depth(const Matrix_MxN &depth, Vector2 uv) {
    int p, q;
    int u = round(uv(0));
    int v = round(uv(1));
    if (depth(u, v) > 0) return depth(u, v);
    for (int offset = 1; offset < 50; ++offset) {

        p = u - offset;
        for (int k = -offset; k <= offset; k++) {
            if (k == 0) continue;
            q = v + k;
            if (p >= 0 && p < depth.rows() && q >= 0 && q < depth.cols()) {
                if (depth(p, q) > 0) return depth(p, q);
            }
        }
        p = u + offset;
        for (int k = -offset; k <= offset; k++) {
            if (k == 0) continue;
            q = v + k;
            if (p >= 0 && p < depth.rows() && q >= 0 && q < depth.cols()) {
                if (depth(p, q) > 0) return depth(p, q);
            }
        }
        q = v - offset;
        for (int k = -offset; k <= offset; k++) {
            if (k == 0) continue;
            p = u + k;
            if (p >= 0 && p < depth.rows() && q >= 0 && q < depth.cols()) {
                if (depth(p, q) > 0) return depth(p, q);
            }
        }
        q = v + offset;
        p = u + offset;
        for (int k = -offset; k <= offset; k++) {
            if (k == 0) continue;
            p = u + k;
            if (p >= 0 && p < depth.rows() && q >= 0 && q < depth.cols()) {
                if (depth(p, q) > 0) return depth(p, q);
            }
        }
    }
    cout << "No data in 50 neighborhood" << endl; return -1;
}

void FindFingers::find_median_loop(const Matrix_MxN &depth, int r, int c, int offset, std::vector<Scalar> &neighbors) {
    neighbors.clear();
    for (int u = -offset; u <= offset; u++) {
        for (int v = -offset; v <= offset; v++) {
            if (r + u < 0 || r + u >= depth.rows() || c + v < 0 || c + v >= depth.cols()) continue;
            if (depth(r + u, c + v) > 0) neighbors.push_back((depth(r + u, c + v)));
        }
    }
}

Scalar FindFingers::get_median_depth(const Matrix_MxN &depth, Vector2 uv) {
    int offset = 4;
    int r = round(uv(0));
    int c = round(uv(1));
    std::vector<Scalar> neighbors;
    do {
        find_median_loop(depth, r, c, offset, neighbors);
        //cout << "offset = " << offset << endl;
        offset++;
    } while (neighbors.size() == 0);
    size_t n = neighbors.size() / 2;
    nth_element(neighbors.begin(), neighbors.begin() + n, neighbors.end());
    return neighbors[n];
}

void FindFingers::feed_results(const Matrix_MxN &depth) {
    if (call_stack) cout << "[FindFingers::feed_results()]" << endl;

    size_t num_targets = xy_finger_tips.size() + z_finger_tips.size() + 1;

    Vector3 p1, p2;
    Eigen::Matrix<Scalar, 6, 1> l = Eigen::Matrix<Scalar, 6, 1>::Zero(6, 1);
    size_t targets_count = 0;
    size_t num_directions = 0;
    Eigen::Matrix<Scalar, Eigen::Dynamic, 3> point_targets = Eigen::Matrix<Scalar, Eigen::Dynamic, 3>::Zero(num_targets, 3);
    Eigen::Matrix<Scalar, Eigen::Dynamic, 6> line_targets = Eigen::Matrix<Scalar, Eigen::Dynamic, 6>::Zero(num_targets, 6);
    Eigen::Matrix<Scalar, 6, 1> plane_targets = Eigen::Matrix<Scalar, 6, 1>::Zero(6);
    Eigen::Matrix<Scalar, Eigen::Dynamic, 6> direction_targets = Eigen::Matrix<Scalar, Eigen::Dynamic, 6>::Zero(num_targets, 6);
    std::vector<size_t> point_targets_indices;
    std::vector<size_t> line_targets_indices;
    std::vector<size_t> direction_targets_indices;

    //cout << "Palm constraints" << endl;
    p1 = worker->camera->depth_to_world(palm_center(1), palm_center(0), palm_center(2));
    p2 = worker->camera->depth_to_world(palm_center(1) + palm_normal(1), palm_center(0) + palm_normal(0), palm_center(2) + palm_normal(2));
    point_targets.row(targets_count) = p1;
    plane_targets.head(3) = p1; plane_targets.tail(3) = p2;
    point_targets_indices.push_back(targets_count);
    targets_count++;

    //cout << "XY fingers constraints" << endl;
    for (size_t i = 0; i < xy_finger_tips.size(); i++) {
        //cout << "before depth" << endl;
        float d1 = get_median_depth(depth, xy_finger_tips[i]);
        float d2 = get_median_depth(depth, xy_finger_roots[i]);
        //cout << "after depth" << endl;
        p1 = worker->camera->depth_to_world(xy_finger_tips[i](1), xy_finger_tips[i](0), d1);
                p2 = worker->camera->depth_to_world(xy_finger_roots[i](1), xy_finger_roots[i](0), d2);

                point_targets.row(targets_count) = p1;
        l.head(3) = p1; l.tail(3) = p2;
        line_targets.row(targets_count) = l;

        point_targets_indices.push_back(targets_count);
        line_targets_indices.push_back(targets_count);
        targets_count++;
    }

    //cout << "Z fingers constraints" << endl;
    for (size_t i = 0; i < z_finger_tips.size(); i++) {
        float d1 = get_median_depth(depth, z_finger_tips[i]);
        float d2 = get_median_depth(depth, z_finger_roots[i]);
        p1 = worker->camera->depth_to_world(z_finger_tips[i](1), z_finger_tips[i](0), d1);
                p2 = worker->camera->depth_to_world(z_finger_roots[i](1), z_finger_roots[i](0), d2);

                if (p1(0) == p2(0) && p1(1) == p2(1) && p1(2) == p2(2)) {
                point_targets.row(targets_count) = p1;
                point_targets_indices.push_back(targets_count);
    }
                else {
                l.head(3) = p1; l.tail(3) = p2;
                line_targets.row(targets_count) = l;
                line_targets_indices.push_back(targets_count);
                direction_targets.row(targets_count) = l;
                direction_targets_indices.push_back(targets_count);
                num_directions++;
    }
                targets_count++;
    }

    detection->set_plane_targets(plane_targets);
    detection->set_point_targets(point_targets);
    detection->set_line_targets(line_targets);
    detection->set_direction_targets(direction_targets);

    detection->set_point_targets_indices(point_targets_indices);
    detection->set_line_targets_indices(line_targets_indices);
    detection->set_direction_targets_indices(direction_targets_indices);

    detection->set_num_directions(num_directions);
    detection->set_num_targets(num_targets);

    if ((xy_finger_tips.size() + z_finger_tips.size() < num_fingers)) return;
    detection->set_radial_order(radial_order);
    detection->set_linear_order(linear_order);
    detection->set_thumb_index(thumb_index);
    detection->set_hand_side(hand_side);
    detection->update_permutations_vector();

    /*cout << endl << "INPUT" << endl << endl;

        cout << "palm_center = " << endl << palm_center.transpose() << endl << endl;
        cout << "palm_normal = " << endl << palm_normal.transpose() << endl << endl;
        cout << "xy_fingers = " << endl;
        for (size_t i = 0; i < xy_finger_tips.size(); i++) {
        cout << xy_finger_tips[i](0) << " " <<  xy_finger_tips[i](1) << " " << get_depth(depth, xy_finger_tips[i]) << " "
        << xy_finger_roots[i](0) << " " <<  xy_finger_roots[i](1) << " "<<get_depth(depth, xy_finger_roots[i]) << endl;
        }
        cout << "z_fingers = " << endl;
        for (size_t i = 0; i < z_finger_tips.size(); i++) {
        cout << z_finger_tips[i](0) << " " <<  z_finger_tips[i](1) << " " << get_depth(depth, z_finger_tips[i]) << " "
        << z_finger_roots[i](0) << " " <<  z_finger_roots[i](1) << " " <<  get_depth(depth, z_finger_roots[i]) << endl;
        }

        cout << endl << "CONSTRAINTS" << endl << endl;

        cout << "plane_target = " << endl << plane_target << endl << endl;
        cout << "point_target = " << endl << point_target << endl << endl;
        cout << "line_target = " << endl << line_target << endl << endl;
        cout << "direction_target = " << endl << direction_target << endl << endl;
        cout << "num_directions = " << endl << num_directions << endl << endl;
        cout << "num_targets = " << endl << num_targets << endl << endl;
        cout << "point_target_indices = " << endl;
        for (int k = 0; k < point_target_indices.size(); ++k) {
        cout << point_target_indices[k] << " ";
        } cout << endl << endl;
        cout << "line_target_indices = " << endl;
        for (int k = 0; k < line_target_indices.size(); ++k) {
        cout << line_target_indices[k] << " ";
        } cout << endl << endl;
        cout << "direction_targets_indices = " << endl;
        for (int k = 0; k < direction_targets_indices.size(); ++k) {
        cout << direction_targets_indices[k] << " ";
        } cout << endl << endl;*/
}

Matrix_MxN FindFingers::process_input() {
    if (call_stack) cout << "[FindFingers::process_input()]" << endl;

    size_t kernel_size = 1;
    float threshold_value = 100;
    size_t threshold_type = 0;
    size_t const max_binary_value = 255;

    worker->handfinder->binary_classification(worker->current_frame);
    cv::Mat mask = worker->handfinder->sensor_silhouette.clone();
    

    cv::GaussianBlur(mask, mask, cv::Size(2 * kernel_size + 1, 2 * kernel_size + 1), 3);
    cv::threshold(mask, mask, threshold_value, max_binary_value, threshold_type);
    cv::Mat erosion_element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2 * kernel_size + 1, 2 * kernel_size + 1), cv::Point(kernel_size, kernel_size));
    cv::erode(mask, mask, erosion_element);

    size_t kernel_width = 2;
    cv::Mat cv_depth = worker->current_frame.depth.clone();
    cv::medianBlur(cv_depth, cv_depth, 1 + kernel_width * 2);

    Matrix_MxN eigen_mask; cv2eigen(mask, eigen_mask);

    Matrix_MxN eigen_depth; cv2eigen(cv_depth, eigen_depth);

    Matrix_MxN depth = Matrix_MxN::Zero(eigen_depth.rows(), eigen_depth.cols());

    for (size_t i = 0; i < eigen_mask.rows(); i++) {
        for (size_t j = 0; j < eigen_mask.cols(); j++) {
            if (eigen_mask(i, j) == 255 && eigen_depth(i, j) < 2000)
                depth(i, j) = eigen_depth(i, j);
        }
    }
    return depth;
}

void FindFingers::find_permutations() {
    if (call_stack) cout << "[FindFingers::find_permutations()]" << endl;

    std::vector<std::pair<Scalar, size_t>> radial;
    std::vector<std::pair<Scalar, size_t>> linear;
    for (size_t i = 0; i < num_fingers; i++) {
        Vector2 v1 = Vector2(0.0, 1.0);
        Vector2 root;
        if (i < xy_finger_roots.size()) root = xy_finger_roots[i];
        else root = z_finger_roots[i - xy_finger_roots.size()];
        Vector2 v2 = root - palm_center.head(2);

        v2 = v2 / v2.norm();
        Scalar a = atan2(v1(0) * v2(1) - v2(0) * v1(1), v1(0) * v2(0) + v1(1) * v2(1));
        Scalar angle = 180.0 * a / M_PI;
        if (angle < 0) angle += 360;
        radial.push_back(std::pair<Scalar, size_t>(angle, i));
        linear.push_back(std::pair<Scalar, size_t>(root(1), i));
    }
    std::sort(radial.begin(), radial.end(), float_comparator);
    std::sort(linear.begin(), linear.end(), float_comparator);

    radial_order.clear();
    linear_order.clear();
    for (size_t i = 0; i < num_fingers; i++) {
        radial_order.push_back(radial[i].second);
        linear_order.push_back(linear[i].second);
    }
    //for (size_t i = 0; i < radial.size(); i++) { cout << radial[i].second << " "; } cout << endl;
    //for (size_t i = 0; i < linear.size(); i++) { cout << linear[i].second << " "; } cout << endl;
}

void FindFingers::find_thumb(const Matrix_MxN &depth) {
    Eigen::Matrix<Scalar, num_fingers, 1> distances = std::numeric_limits<Scalar>::max() *
            Eigen::Matrix<Scalar, num_fingers, 1>::Ones(num_fingers, 1);

    std::vector<Vector2> tips;
    for (size_t i = 0; i < xy_finger_tips.size(); i++)
        tips.push_back(xy_finger_tips[i]);
    for (size_t i = 0; i < z_finger_tips.size(); i++)
        tips.push_back(z_finger_tips[i]);

    for (size_t i = 0; i < num_fingers; i++) {
        for (size_t j = 0; j < num_fingers; j++) {
            if (i == j) continue;
            Scalar distance = (tips[i] - tips[j]).norm();

            if (distance < distances(i)) {
                distances(i) = distance;
            }
        }
    }
    Scalar max_distance = distances.maxCoeff(&thumb_index);
    distances(thumb_index) = 0;
    Scalar second_distance = distances.maxCoeff();
    Scalar distance_ratio = second_distance / max_distance;

    if (distance_ratio > 0.80) {
        thumb_index = -1;
        return;
    }


    // Find hand side
    wristband_center = camera->world_to_image(worker->handfinder->wristband_center());
    Vector2 wristband_shift = camera->world_to_image(worker->handfinder->wristband_center() + worker->handfinder->wristband_direction());
    wristband_center = Vector2(depth.rows() - wristband_center(1), wristband_center(0));
    wristband_shift = Vector2(depth.rows() - wristband_shift(1), wristband_shift(0));
    wristband_direction = wristband_shift - wristband_center;
    wristband_direction /= wristband_direction.norm();

    thumb_tip = tips[thumb_index];
    Vector2 x = thumb_tip - wristband_center;
    Vector2 y = wristband_direction;

    Scalar a = x(0) * y(1) - x(1) * y(0);

    if (a < 0) {
        hand_side = 0;
        //cout << "BACK SIDE" << endl;
    }
    else {
        hand_side = 1;
        //cout << "FONT SIDE" << endl;
    }
}

bool FindFingers::find_fingers_main(bool display) {
    if (call_stack) cout << "[FindFingers::fing_fingers_main()]" << endl;
    Matrix_MxN original_depth = process_input();
    //eigen_imshow(original_depth, "mask"); cv::waitKey(5);

    if (matlab::nnz(original_depth) == 0) {
        cout << "Hand is not found" << endl;
        return false;
    }

    Matrix_MxN depth = crop_image(original_depth);
    depth = downsample_image(depth);
    {
        M = depth.rows();
        N = depth.cols();
        num_of_pixels_in_hand = matlab::nnz(depth);
        Matrix_MxN xy_depth = depth;
        Matrix_MxN z_depth = depth;

        if (!find_xy_fingers(xy_depth)) return false;
        if (!find_z_fingers(xy_depth, z_depth)) return false;

        find_finger_directions(XY);
        find_finger_directions(Z);
        find_palm(xy_depth, z_depth);
    }
    restore_original_image();

    if (xy_finger_tips.size() + z_finger_tips.size() == num_fingers) {
        find_thumb(original_depth);
        find_permutations();

    }
    if(display)
        display_all_fingers(original_depth);
    feed_results(original_depth);

    if (detection->get_num_targets() == num_fingers + 1 && detection->get_thumb_index() >= 0)
        return true;
    else return false;
}






