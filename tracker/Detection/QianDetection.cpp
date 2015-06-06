#include <fstream>
#include <math.h>
#include "util/gl_wrapper.h"
#include "util/opencv_wrapper.h"

#include "tracker/Worker.h"
#include "tracker/DataStructure/SkeletonSerializer.h"
#include "tracker/OpenGL/DebugRenderer/DebugRenderer.h"
#include "tracker/HandFinder/HandFinder.h"
#include "tracker/TwSettings.h"

///--- Locals
#include "DetectionStream.h"
#include "QianDetection.h"
#include "FindFingers.h"

QianDetection::QianDetection(Worker *worker) :
    worker(worker),
    skeleton(worker->skeleton)
{

    detection = new DetectionStream(worker->cylinders, worker->skeleton);
    find_fingers = new FindFingers(worker, detection);

    tw_settings->tw_add(settings->display, "Detect. SHOW?","group=Tracker");
}

QianDetection::~QianDetection(){
    delete detection;
    delete find_fingers;
}

bool QianDetection::can_reinitialize(){
    return find_fingers->find_fingers_main(settings->display);
}

void QianDetection::reinitialize(){
    this->detection_iterate();
}

bool verbose = false;


void QianDetection::draw_detected_features(bool verbose){

	if (!verbose) return;

	//Draw fingers
    DebugRenderer::instance().clear();
	std::vector<Vector3> points;
	std::vector<std::pair<Vector3, Vector3>> lines;

	std::vector<size_t> indices;
	indices = detection->get_point_targets_indices();
	for (int i = 0; i < indices.size(); ++i) {
		size_t index = indices[i];
		Vector3 target = detection->get_point_target(index);
		points.push_back(target);
	}
	indices = detection->get_line_targets_indices();
	for (int i = 0; i < indices.size(); ++i) {
		size_t index = indices[i];
		Vector3 start = detection->get_line_target_start(index);
		Vector3 end = detection->get_line_target_end(index);
		lines.push_back(std::pair<Vector3, Vector3>(start, end));
	}

	//Draw palm
	/*Vector3 t = detection->get_plane_target_origin();
	Vector3 n = detection->get_plane_target_normal();
	Vector3 d1 = Vector3(n(0), -n(1), 0);
	d1.normalize();
	Vector3 d2 = n.cross(d1);
	d2.normalize();
	Scalar factor = 35;
	lines.push_back(std::pair<Vector3, Vector3>(t, t + factor * d1));
	lines.push_back(std::pair<Vector3, Vector3>(t, t - factor * d1));
	lines.push_back(std::pair<Vector3, Vector3>(t, t + factor * d2));
	lines.push_back(std::pair<Vector3, Vector3>(t, t - factor * d2));*/

    DebugRenderer::instance().add_points(points, Vector3(0, 1, 0));
    DebugRenderer::instance().add_segments(lines, Vector3(1, 0.5, 0));
    worker->updateGL();
}


void QianDetection::draw_point_constraints(size_t permutation, size_t index, bool verbose) {
	if (!verbose) return;

    //DebugRenderer::instance().clear();
	std::vector<std::pair<Vector3, Vector3>> lines;
	std::vector<size_t> ids = detection->get_ids(permutation, index);
	Vector3 s = detection->get_point_source(ids[0]);
	Vector3 t = detection->get_point_target(index);
	lines.push_back(std::pair<Vector3, Vector3>(s, t));
    DebugRenderer::instance().add_segments(lines, Vector3(0, 0.8, 0.5));
    worker->updateGL();
}


void QianDetection::draw_line_constraints(size_t permutation, size_t index, size_t j, Vector3 p, bool verbose) {
	if (!verbose) return;

    //DebugRenderer::instance().clear();
	std::vector<std::pair<Vector3, Vector3>> lines;

	std::vector<size_t> ids = detection->get_ids(permutation, index);
	Vector3 s = detection->get_line_source(ids[j]);
	lines.push_back(std::pair<Vector3, Vector3>(s, p));
    DebugRenderer::instance().add_segments(lines, Vector3(1, 0, 0));
    worker->updateGL();
}


void QianDetection::draw_plane_constraints(size_t j, Vector3 p, bool verbose) {
	if (!verbose) return;

    //DebugRenderer::instance().clear();
	std::vector<std::pair<Vector3, Vector3>> lines;
	Vector3 s = detection->get_plane_source(j);
	lines.push_back(std::pair<Vector3, Vector3>(s, p));
    DebugRenderer::instance().add_segments(lines, Vector3(0, 0, 1));
    worker->updateGL();
}


void QianDetection::draw_direction_constraints(size_t permutation, size_t direction, size_t index, size_t i, bool verbose) {
	if (!verbose) return;

    //DebugRenderer::instance().clear();
	std::vector<std::pair<Vector3, Vector3>> lines;
	std::vector<size_t> ids = detection->get_ids(permutation, index);
	Vector3 s = detection->get_point_source(ids[0]);
	Vector3 t = detection->get_direction_target(direction, index, i);
	lines.push_back(std::pair<Vector3, Vector3>(s, t));
    DebugRenderer::instance().add_segments(lines, Vector3(0.7, 0, 0.8));
    worker->updateGL();
}


void QianDetection::detect_compute_points_update(LinearSystem& system_detection, size_t permutation, bool verbose) {
	if (call_stack) cout << "[Worker_detection::detect_compute_points_update()]" << endl;

	const int k = detection->get_num_point_targets();

	Scalar c = 3;

	Eigen::Matrix<Scalar, Eigen::Dynamic, num_thetas> J = Eigen::Matrix<Scalar, Eigen::Dynamic, num_thetas>::Zero(c * k, num_thetas);
	VectorN f = VectorN::Zero(c * k);

	std::vector<size_t> indices = detection->get_point_targets_indices();
	for (int i = 0; i < k; ++i) {
		size_t index = indices[i];
		Vector3 t = detection->get_point_target(index);
		size_t id = detection->get_ids(permutation, index)[0];
		Vector3 s = detection->get_point_source(id);

		Eigen::Matrix<Scalar, 3, num_thetas> js = Eigen::Matrix<Scalar, 3, num_thetas>::Zero();
        skeleton->jacobian(js, s, id);

		J.block(c * i, 0, c, num_thetas) += js;
		f.block(c * i, 0, c, 1) += factor *(t - s);

		draw_point_constraints(permutation, index, verbose);
	}
	system_detection.lhs.selfadjointView<Eigen::Upper>().rankUpdate(J.transpose(), 3.0);
	system_detection.rhs.noalias() += 3 * J.transpose() * f;
}


void QianDetection::detect_compute_lines_update(LinearSystem& system_detection, size_t permutation, bool verbose) {
	if (call_stack) cout << "[Worker_detection::detect_compute_lines_update()]" << endl;

	const int k = detection->get_num_line_targets();
	Scalar c = 3;

	Eigen::Matrix<Scalar, Eigen::Dynamic, num_thetas> J = Eigen::Matrix<Scalar, Eigen::Dynamic, num_thetas>::Zero(2 * c * k, num_thetas);
	VectorN f = VectorN::Zero(2 * c * k);

	std::vector<size_t> indices = detection->get_line_targets_indices();

	for (int i = 0; i < k; ++i) {
		size_t index = indices[i];

		std::vector<size_t> ids = detection->get_ids(permutation, index);

		for (int j = 0; j < 3; ++j) {
			size_t id = ids[j];
			Vector3 t = detection->get_line_target_start(index);
			Vector3 r = detection->get_line_target_end(index);
			Vector3 s = detection->get_line_source(id);
			Vector3 n = detection->get_line_target_normal(index);
			Scalar indicator = (r - t).dot(s - t) / (r - t).norm() / (r - t).norm();

			Eigen::Matrix<Scalar, 3, num_thetas> js = Eigen::Matrix<Scalar, 3, num_thetas>::Zero();
            skeleton->jacobian(js, s, id);
			//Matrix_MxN js = worker->jacobian(s, id);

			Vector3 p;
			if (indicator > 0 && indicator < 1) {
				//cout << "case 1" << endl;
				J.block(2 * c * i + j, 0, c, num_thetas) += js - n * n.transpose() * js;
				f.block(2 * c * i + j, 0, c, 1) += factor * ((t - s) - n * n.transpose() * (t - s));
				p = t + n * n.transpose() * (s - t);
			}
			else if (indicator <= 0) {
				//cout << "case 2" << endl;
				J.block(2 * c * i + j, 0, c, num_thetas) += js;
				f.block(2 * c * i + j, 0, c, 1) += factor * (t - s);
				p = t;
			}
			else if (indicator >= 1) {
				//cout << "case 3" << endl;
				J.block(2 * c * i + j, 0, c, num_thetas) += js;
				f.block(2 * c * i + j, 0, c, 1) += factor * (r - s);
				p = r;
			}
			draw_line_constraints(permutation, index, j, p, verbose);
		}
	}
	system_detection.lhs.selfadjointView<Eigen::Upper>().rankUpdate(J.transpose(), 1.0);
	system_detection.rhs.noalias() += J.transpose() * f;
}


void QianDetection::detect_compute_planes_update(LinearSystem& system_detection, bool verbose) {
	if (call_stack) cout << "[Worker_detection::detect_compute_planes_update()]" << endl;

	int h = detection->get_num_plane_targets();
	Scalar c = 3;
	Eigen::Matrix<Scalar, Eigen::Dynamic, num_thetas> J = Eigen::Matrix<Scalar, Eigen::Dynamic, num_thetas>::Zero(c * h, num_thetas);
	VectorN f = VectorN::Zero(c * h);

	size_t id = detection->get_plane_id();
	for (int j = 0; j < h; ++j) {
		Vector3 t = detection->get_plane_target_origin();
		Vector3 s = detection->get_plane_source(j);
		Vector3 n = detection->get_plane_target_normal();

		Vector3 p = s - n * n.transpose() * (s - t);

		Eigen::Matrix<Scalar, 3, num_thetas> js = Eigen::Matrix<Scalar, 3, num_thetas>::Zero();
        skeleton->jacobian(js, s, id);
		J.block(c * j, 0, c, num_thetas) += -n * n.transpose() * js;
		f.block(c * j, 0, c, 1) += 0.4 * factor * (n * n.transpose() * (s - t));

		draw_plane_constraints(j, p, verbose);

		/*cout << "s = " << s.transpose() << endl;
		cout << "t = " << t.transpose() << endl;
		cout << "n = " << n.transpose() << endl;
		cout << "n * n.transpose() * (s - t) = " << (n * n.transpose() * (s - t)).transpose() << endl << endl;
		std::vector<std::pair<Vector3, Vector3>> lines;
		std::vector<Vector3> colors;
		lines.push_back(std::pair<Vector3, Vector3>(t,  t + n * n.transpose() * (s - t)));
		colors.push_back(Vector3(1, 0, 0));
		lines.push_back(std::pair<Vector3, Vector3>(t, s));
		colors.push_back(Vector3(1, 0, 1));
		lines.push_back(std::pair<Vector3, Vector3>(t + n * n.transpose() * (s - t), s));
		colors.push_back(Vector3(1, 1, 0));
        DebugRenderer::instance().add_segments(lines, colors);
		::glarea->updateGL();*/
	}

	system_detection.lhs.selfadjointView<Eigen::Upper>().rankUpdate(J.transpose(), 10.0);
	system_detection.rhs.noalias() += J.transpose() * f;
}


void QianDetection::detect_compute_direction_update(LinearSystem& system_detection, size_t permutation, size_t direction, bool verbose) {
	if (call_stack) cout << "[detect_compute_directions_update()]" << endl;

	const int k = detection->get_num_direction_targets();
	Scalar c = 3;

	Eigen::Matrix<Scalar, Eigen::Dynamic, num_thetas> J = Eigen::Matrix<Scalar, Eigen::Dynamic, num_thetas>::Zero(c * k, num_thetas);

	VectorN f = VectorN::Zero(c * k);

	std::vector<size_t> indices = detection->get_direction_targets_indices();
	for (int i = 0; i < k; ++i) {
		size_t index = indices[i];
		Vector3 t = detection->get_direction_target(direction, index, i);
		size_t id = detection->get_ids(permutation, index)[0];
		Vector3 s = detection->get_point_source(id);

		//Matrix_MxN js = worker->jacobian(s, id);
		Eigen::Matrix<Scalar, 3, num_thetas> js = Eigen::Matrix<Scalar, 3, num_thetas>::Zero();
        skeleton->jacobian(js, s, id);

		J.block(c * i, 0, c, num_thetas) += js;
		f.block(c * i, 0, c, 1) += factor *(t - s);

		draw_direction_constraints(permutation, direction, index, i, verbose);
	}
	system_detection.lhs.selfadjointView<Eigen::Upper>().rankUpdate(J.transpose(), 1.0);
	system_detection.rhs.noalias() += J.transpose() * f;
}


Scalar QianDetection::compute_detection_energy(size_t permutation, size_t direction) {
	//cout << "[compute_detection_energy()]" << endl;

	Scalar energy = 0;

	// Point constraints energy
	std::vector<size_t> point_targets_indices = detection->get_point_targets_indices();
	for (int i = 0; i < detection->get_num_point_targets(); ++i) {
		size_t index = point_targets_indices[i];
		size_t id = detection->get_ids(permutation, index)[0];
		Vector3 t = detection->get_point_target(index);
		Vector3 s = detection->get_point_source(id);
		energy += (t - s).transpose() * (t - s);
	}

	// Line constraints energy
	std::vector<size_t> line_targets_indices = detection->get_line_targets_indices();
	for (int i = 0; i < detection->get_num_line_targets(); ++i) {
		size_t index = line_targets_indices[i];
		std::vector<size_t> ids = detection->get_ids(permutation, index);
		for (int j = 0; j < 2; ++j) {
			size_t id = ids[j];
			Vector3 t = detection->get_line_target_start(index);
			Vector3 r = detection->get_line_target_end(index);
			Vector3 s = detection->get_line_source(id);
			Vector3 n = detection->get_line_target_normal(index);
			Scalar indicator = (r - t).dot(s - t) / (r - t).norm() / (r - t).norm();
			if (indicator > 0 && indicator < 1) {
				Vector3 e = (t - s) - n * n.transpose() * (t - s);
				energy += e.transpose() * e;
			}
			else if (indicator <= 0) {
				energy += (t - s).transpose() * (t - s);
			}
			else if (indicator >= 1) {
				energy += (r - s).transpose() * (r - s);
			}
		}
	}

	// Plane constraints energy
	for (int j = 0; j < detection->get_num_plane_targets(); ++j) {
		Vector3 t = detection->get_plane_target_origin();
		Vector3 s = detection->get_plane_source(j);
		Vector3 n = detection->get_plane_target_normal();
		Vector3 e = n * n.transpose() * (s - t);
		energy += e.transpose() * e;
	}

	// Direction constraints energy
	std::vector<size_t> direction_targers_indices = detection->get_direction_targets_indices();
	for (int i = 0; i < detection->get_num_direction_targets(); ++i) {
		size_t index = direction_targers_indices[i];
		size_t id = detection->get_ids(permutation, index)[0];
		Vector3 t = detection->get_direction_target(direction, index, i);
		Vector3 s = detection->get_point_source(id);
		energy += (t - s).transpose() * (t - s);
	}
	return energy;
}


void QianDetection::my_damping_track(LinearSystem& system) {
	Scalar lambda = 300;
	Scalar lambda_trans = 1;

	Mapping mapping = worker->skeleton->getMapping();
	const int num = mapping.getDimension();
	Eigen::Matrix<Scalar, num_thetas, num_thetas> D = Eigen::Matrix<Scalar, num_thetas, num_thetas>::Identity(system.lhs.rows(), system.lhs.cols());
	Eigen::Matrix<Scalar, num_thetas, 1>  d = Eigen::Matrix<Scalar, num_thetas, 1>::Ones(system.lhs.rows(), 1);

	for (int i = 0; i < num; ++i) {
		if (mapping.getJointInfo(i).type == TRANSLATION_AXIS)
			d(i) = lambda_trans;
		else
			d(i) = lambda;
		if (i >= 3 && i < 6)
			d(i) *= 1000;
	}
	D.diagonal() = d;
	system.lhs = system.lhs + D;
}


void QianDetection::optimize_current_permutation(size_t permutation, size_t direction, Eigen::Matrix<Scalar, num_thetas, 1> theta_0, bool verbose) {
	if (call_stack) cout << "[Worker_detection::optimize_current_permutation()]" << endl;
	skeleton->set(theta_0);
    skeleton->update();
	Eigen::Matrix<Scalar, num_thetas, 1> theta = theta_0;

	for (int optimization_iteration = 0; optimization_iteration < 15; optimization_iteration++) {
		//cout << endl << "optimization_iteration = " << optimization_iteration << endl;
		skeleton->set(theta);
        skeleton->update();
		/*if (verbose) {
			render_offscreen();
			glarea->updateGL();
		}*/

		LinearSystem system_detection(num_thetas);
		system_detection.lhs.setZero();
		system_detection.rhs.setZero();
		draw_detected_features(verbose);
		detect_compute_points_update(system_detection, permutation, verbose);
		detect_compute_lines_update(system_detection, permutation, verbose);
		detect_compute_direction_update(system_detection, permutation, direction, verbose);
		detect_compute_planes_update(system_detection, verbose);

		std::vector<Scalar> theta_std;
		for(int t = 0; t < theta.size(); ++t) {
			theta_std.push_back(theta(t));
		}
        worker->E_limits.track(system_detection, theta_std);
        my_damping_track(system_detection);
        worker->E_wristband.track(system_detection);
        // worker->E_pose.track(system_detection, theta_std);

		VectorN delta_theta = system_detection.lhs.colPivHouseholderQr().solve(system_detection.rhs);
        std::vector<Scalar> delta_theta_std;
        for(int t = 0; t < delta_theta.size(); ++t)
            delta_theta_std.push_back(delta_theta(t));

		//cout << "delta_theta.norm() = " << delta_theta.norm() << endl;
		/*ostringstream convert; convert << optimization_iteration;
		std::ofstream("/home/tkach/Desktop/LHS_" + convert.str() + ".txt") << system_detection.lhs;
		std::ofstream("/home/tkach/Desktop/rhs" + convert.str() + ".txt") << system_detection.rhs;
		std::ofstream("/home/tkach/Desktop/delta_theta" + convert.str() + ".txt") << delta_theta;*/

#if 1
        theta_std = skeleton->getUpdatedParameters(theta_std, delta_theta_std);
        theta = Eigen::Map<Thetas>(theta_std.data());
#else
        // theta = theta + delta_theta.head(num_thetas);
#endif
	}
}


void QianDetection::rigid_aligment(const Eigen::Matrix<Scalar, num_thetas, 1> & theta_0, Eigen::Matrix<Scalar, num_thetas, 1> & theta, Vec3f & euler_angles, size_t permutation_index) {
	Eigen::Matrix<Scalar, 3, Eigen::Dynamic> Vs =
		Eigen::Matrix<Scalar, 3, Eigen::Dynamic>::Zero(3, detection->get_num_point_targets() + 1);
	Eigen::Matrix<Scalar, 3, Eigen::Dynamic> Vt =
		Eigen::Matrix<Scalar, 3, Eigen::Dynamic>::Zero(3, detection->get_num_point_targets() + 1);

	worker->skeleton->set(theta_0);
    worker->skeleton->update();

	Scalar palm_length = worker->cylinders->getSegmentByID(worker->skeleton->getID("Hand")).length;
    Vector3 palm_root_target = detection->get_point_target(0) - 0.5 * palm_length * worker->handfinder->wristband_direction();
	Vector3 palm_root_source = worker->skeleton->getJoint(worker->skeleton->getID("Hand"))->getGlobalTranslation();
	Vs.col(detection->get_num_point_targets()) = palm_root_source.transpose();
	Vt.col(detection->get_num_point_targets()) = palm_root_target.transpose();

	std::vector<size_t> indices = detection->get_point_targets_indices();
	for (int i = 0; i < indices.size(); ++i) {
		size_t index = indices[i];
		Vector3 t = detection->get_point_target(index);
		size_t id = detection->get_ids(permutation_index, index)[0];
		Vector3 s = detection->get_point_source(id);
		Vs.col(i) = s.transpose();
		Vt.col(i) = t.transpose();
	}
	Mat4f T = Eigen::umeyama(Vs, Vt, false);
	Mat3f rotation = T.block(0, 0, 3, 3);
    // Vec3f translation = T.block(0, 3, 3, 1);
	euler_angles = worker->skeleton->getMapping().toEulerAngles(rotation);


	worker->skeleton->set(theta_0);
	worker->skeleton->transform(T);
	theta = worker->skeleton->get();

	// Find rigid fitting error
	/*Vs = rotation * Vs;
	Vs.colwise() += translation;
	Eigen::Matrix<Scalar, num_fingers, 1>  norms = (Vs - Vt).colwise().norm();
	Scalar rigid_error = norms.maxCoeff();
	return rigid_error;*/

	// Display
	/*std::vector<Vector3> result_points;
	for (int i = 0; i < num_fingers + 2; ++i) result_points.push_back(Vs.col(i));
    DebugRenderer::instance().add_points(result_points, Vector3(0, 1, 0));
	std::vector<Vector3> target_points;
	for (int i = 0; i < num_fingers + 2; ++i) target_points.push_back(Vt.col(i));
    DebugRenderer::instance().add_points(target_points, Vector3(1, 0, 0));
	std::vector<std::pair<Vector3, Vector3>> lines;
	for (int i = 0; i < num_fingers + 2; ++i) lines.push_back(std::pair<Vector3, Vector3>(Vt.col(i), Vs.col(i)));
    DebugRenderer::instance().add_segments(lines, Vector3(0, 0.8, 0.5));
	::glarea->updateGL();*/

}


void QianDetection::detection_iterate() {
	if (call_stack) cout << "[Worker_detection::detection_iterate()]" << endl;


	// Set palm position
	Eigen::Matrix<Scalar, num_thetas, 1> theta_0 = Eigen::Matrix<Scalar, num_thetas, 1>::Zero(num_thetas, 1);
	Mat3f rotation = worker->skeleton->getInitialTransformation("Hand").block(0, 0, 3, 3);
	Segment segment = worker->cylinders->getSegmentByID(worker->skeleton->getID("Hand"));
	Vector3 shift = 0.5 * rotation * segment.length *  Vector3(0, 1, 0);
	Vector3 target = detection->get_plane_target_origin() - shift;
	theta_0.head(3) = target;

	if (detection->get_num_targets() < num_fingers + 1 || detection->get_thumb_index() < 0) {
		return;
	}

	// Find min rigid error pose
	size_t min_energy_permutation_index;
	size_t min_energy_direction_index;

	//system_detection = LinearSystem(num_thetas);
	Scalar min_energy = std::numeric_limits<Scalar>::max();

	Vec3f min_euler_angles;

	Eigen::Matrix<Scalar, num_thetas, 1> theta_min_energy;
	Eigen::Matrix<Scalar, num_thetas, 1> theta;


	// Fit the fingers
	for (int direction = 0; direction < max(detection->get_num_directions(), (size_t)1); direction++) {
		for (int permutation_index = 0; permutation_index < detection->get_num_permutations(); permutation_index++) {
			//cout << "permutation [" << permutation_index << "] = " << detection->get_permutation(permutation_index).transpose() << endl;

			//Rigid alignment
			Vec3f euler_angles;
			rigid_aligment(theta_0, theta, euler_angles, permutation_index);

			optimize_current_permutation(permutation_index, direction, theta, false);
			Scalar energy = compute_detection_energy(permutation_index, direction);
			if (energy < min_energy) {
				min_energy = energy;
				min_energy_permutation_index = permutation_index;
				min_energy_direction_index = direction;
				theta_min_energy = theta;
				min_euler_angles = euler_angles;
			}
		}
		break;
	}
	//cout << "best permutation = " << detection->get_permutation(min_energy_permutation_index).transpose() << endl;
	//cout << "min_energy_permutation_index = " << min_energy_permutation_index << endl;
	//if (detection->get_num_directions()) cout << "best direction = " << detection->get_direction(min_energy_direction_index).transpose() << endl;

	optimize_current_permutation(min_energy_permutation_index, min_energy_direction_index, theta_min_energy, verbose);

}




