#pragma once
#include "tracker/ForwardDeclarations.h"
#include "tracker/Types.h"

class QianDetection {
private:
	Worker*const worker;
    SkeletonSerializer*const skeleton;
	FindFingers* find_fingers;
	DetectionStream* detection;
	Scalar factor = 0.5;
	const bool call_stack = false;

public:
    struct Settings{
        bool display = true;
    } _settings;
    Settings*const settings=&_settings;

public:
	QianDetection(Worker* worker);
	~QianDetection();
public:
	bool can_reinitialize();
	void reinitialize();

private:
	void detection_iterate();
	void rigid_aligment(const Eigen::Matrix<Scalar, num_thetas, 1>& theta_0,
		Eigen::Matrix<Scalar, num_thetas, 1> & theta,
		Vector3& euler_angles,
		size_t permutation_index);
	void my_damping_track(LinearSystem &system);
	void draw_detected_features(bool verbose);
	void draw_point_constraints(size_t permutation, size_t index, bool verbose);
	void draw_line_constraints(size_t permutation, size_t index, size_t j, Vector3 p, bool verbose);
	void draw_plane_constraints(size_t j, Vector3 p, bool verbose);
	void draw_direction_constraints(size_t permutation, size_t direction, size_t index, size_t i, bool verbose);
	void detect_compute_points_update(LinearSystem &system_detection, size_t permutation, bool verbose);
	void detect_compute_lines_update(LinearSystem &system_detection, size_t permutation, bool verbose);
	void detect_compute_planes_update(LinearSystem &system_detection, bool verbose);
	void detect_compute_direction_update(LinearSystem &system_detection, size_t permutation, size_t direction, bool verbose);
	Scalar compute_detection_energy(size_t permutation, size_t direction);
	void optimize_current_permutation(size_t permutation, size_t direction, Eigen::Matrix<Scalar, num_thetas, 1> theta_0, bool verbose);
};
