#pragma once
#include "tracker/Types.h"
#include <vector>
#include <set>
#include "tracker/ForwardDeclarations.h"
class DetectionStream;

///@{ TODO: these should not be here but in the C++ file
typedef Eigen::Matrix<size_t, 2, 1> Vector2s;
typedef Eigen::Matrix<size_t, Eigen::Dynamic, 1> VectorNs;
typedef Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> Matrix_MxN_b;
///@}

class FindFingers {
public:
    FindFingers(Worker*worker, DetectionStream* detection);
    bool find_fingers_main(bool display);
private:
    Worker*const worker;
    DetectionStream*const detection;
    Camera*const camera;
    bool call_stack = false;
private:
    float finger_ratio_threshold = 2.0;
	int number_of_hist_bins = 16;
	int model_finger_length = 60;
	int min_number_of_pixels_in_finger = 50;
	float max_fraction_of_pixels_in_finger = 0.3f;
	int start_histogram_mean = 3 - 1;
	float end_histogram_mean = (float)round(number_of_hist_bins / 2.0);
	int local_min_window = 7;
	int model_phalangue_length = 30;
	int finger_radius_threshold = 8; /*8.5*/
	float distribution_ratio_threshold = 0.9f;
	int min_finger_segment_length = 12;
	bool downsampled = false;
	enum Mode { XY, Z };
	std::vector<size_t> radial_order;
	std::vector<size_t> linear_order;
	std::vector<Matrix_MxN> xy_finger_segments;
	std::vector<Matrix_MxN> z_finger_segments;
	std::vector<Vector2> xy_finger_tips;
	std::vector<Vector2> z_finger_tips;
	std::vector<Vector2> xy_finger_roots;
	std::vector<Vector2> z_finger_roots;
	Vector3 palm_center;
	Vector3 palm_normal;
	Matrix_MxN xy_depth;
	Matrix_MxN z_depth;
	size_t M;
	size_t N;
	size_t num_of_pixels_in_hand;
	size_t min_column;
	size_t max_column;
	size_t min_row;
	size_t max_row;
	int thumb_index;
	Vector2 thumb_tip;
	Vector2 wristband_center;
	Vector2 wristband_direction;
	int hand_side;

private:
	typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 2> Matrix_Nx2;
	typedef Eigen::Matrix<Scalar, 2, 2> Matrix_2x2;
	void eigen_imshow(Matrix_MxN eigen_image, string window_name);
	void print_queue(const Matrix_MxN & depth, std::set<std::pair<float, size_t>> queue);
	Matrix_MxN crop_image(const Matrix_MxN & depth);
	void restore_original_image();
	Matrix_MxN downsample_image(const Matrix_MxN & depth);
	void display_all_fingers(const Matrix_MxN & depth);
	void get_neighbors(const Matrix_MxN & depth, const Matrix_MxN_b & mask, Matrix_MxN & distance, Matrix_MxN & visited,
		std::set<std::pair<float, size_t>> & queue, Vector2s v0, Mode mode);
	void dijkstra(const Matrix_MxN & depth, Matrix_MxN & distance, Vector2s v0, float threshold, Mode mode);
	void get_neighbors_vector(const Matrix_MxN & depth, const Matrix_MxN_b & mask, Matrix_MxN & distance, Matrix_MxN & visited,
		std::vector<std::pair<float, size_t>> & queue, Vector2s v0, Mode mode);
	void dijkstra_vector(const Matrix_MxN & depth, Matrix_MxN & distance, Vector2s v0, float threshold, Mode mode);
	void crop_finger(const Matrix_MxN & depth, Matrix_MxN & finger);
	std::set<std::pair<float, size_t>> find_local_min(const Matrix_MxN & depth);
	void compute_finger_pixels_svd(const Matrix_MxN & finger, Matrix_Nx2& data, Eigen::JacobiSVD<Matrix_2x2> & svd,
		float & mean_rows, float & mean_columns);
	bool verify_finger_shape(const Matrix_MxN & finger, Vector2s v, Mode mode);
	bool find_xy_fingers(Matrix_MxN & depth);
	bool find_z_fingers(const Matrix_MxN & xy_depth, Matrix_MxN & z_depth);
	void find_finger_directions(Mode mode);
	void find_palm(const Matrix_MxN & xy_depth, const Matrix_MxN & z_depth);
	float get_depth(const Matrix_MxN & depth, Vector2 uv);
	void find_median_loop(const Matrix_MxN & depth, int r, int c, int offset, std::vector<Scalar> & neighbors);
	Scalar get_median_depth(const Matrix_MxN & depth, Vector2 uv);
	void feed_results(const Matrix_MxN & depth);
	Matrix_MxN process_input();
	void find_permutations();
	void find_thumb(const Matrix_MxN & depth);
};
