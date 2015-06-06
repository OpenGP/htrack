#pragma once
#include <vector>
#include <QString>
#include <fstream>
#include <stdio.h>
#include <bitset>
#include "tracker/Types.h"

#include "tracker/Worker.h"
#include "tracker/Data/DataStream.h"
#include "tracker/DataStructure/SkeletonSerializer.h"
#include "tracker/Legacy/geometry/Joint.h"
#include "tracker/Legacy/geometry/Skeleton.h"
#include "tracker/Legacy/geometry/Cylinders.h"

inline bool size_t_comparator(std::pair<size_t, size_t> a, std::pair<size_t, size_t> b) {
    return a.first < b.first;
}


const size_t factorial[num_fingers + 1] = {1, 1, 2, 6, 24, 120};

struct PalmIndices {
    int root;
    PalmIndices() { }
    PalmIndices(int root){
        this->root = root;
    }
};
struct FingerIndices {
    int root; int mid1; int mid2; int tip;
    FingerIndices() { }
    FingerIndices(int root, int mid1, int mid2, int tip){
        this->root = root; this->mid1 = mid1;
        this->mid2 = mid2; this->tip = tip;
    }
};
struct JointIndices {
    PalmIndices palm;
    FingerIndices thumb;
    FingerIndices pinky;
    FingerIndices ring;
    FingerIndices middle;
    FingerIndices index;
    JointIndices() { }
    JointIndices(PalmIndices palm, FingerIndices thumb, FingerIndices pinky,
                 FingerIndices ring, FingerIndices middle, FingerIndices index){
        this->palm = palm;
        this->thumb = thumb;
        this->pinky = pinky;
        this->ring = ring;
        this->middle = middle;
        this->index = index;
    }
};

class DetectionStream {
    Cylinders*const cylinders;
    Skeleton*const skeleton;

    const bool call_stack = false;

    enum Dataset{
        INVALID,
        Chen,
        Tang,
        Tompson
    };

    JointIndices joint_indices;
    Dataset dataset;
    Vector3 up_direction = Vector3(0, 1, 0);
    Vector3 in_direction = Vector3(0, 0, 1);
    Vector3 side_direction = Vector3(1, 0, 0);

    Matrix_MxN point_targets;
    Matrix_MxN line_targets;
    VectorN plane_targets;
    Matrix_MxN direction_targets;
    std::vector<size_t> point_targets_indices;
    std::vector<size_t> line_targets_indices;
    std::vector<size_t> direction_targets_indices;
    size_t num_targets;
    size_t num_directions;

    std::vector<size_t> radial_order;
    std::vector<size_t> linear_order;

    int thumb_index;
    int hand_side;
    size_t num_permutations;

public:

    std::vector<Matrix_MxN> permutations_vector;
    std::vector<Matrix_MxN> directions_vector;



    DetectionStream(Cylinders* cylinders, Skeleton* skeleton) :
        cylinders(cylinders), skeleton(skeleton)
    {

        PalmIndices palm = PalmIndices(3);

        FingerIndices thumb = FingerIndices(4, 5, 6, 7);
        FingerIndices pinky = FingerIndices(8, 9, 10, 11);
        FingerIndices ring = FingerIndices(12, 13, 14, 15);
        FingerIndices middle = FingerIndices(16, 17, 18, 19);
        FingerIndices index = FingerIndices(20, 21, 22, 23);
        joint_indices = JointIndices(palm, thumb, pinky, ring, middle, index);

        for (size_t i = 0; i < num_fingers; i++) {
            permutations_vector.push_back(Matrix_MxN::Zero( 2 * (num_fingers + 1), i + 1));
        }
        initialize_directions_vector();

#if 0
        if (datastream->is_chen()){
            dataset = Chen;
        }
        else if (datastream->is_tang()){
            dataset = Tang;
        }
        else if (datastream->is_tompson()){
            dataset = Tompson;
        }
        else{
            cout << "[DetectionStream.h,109]: No groundtruth for this dataset" << endl;
        }
#endif
    }

    void initialize_permutations_vector() {
        std::vector<size_t> permutations_counts = std::vector<size_t>(num_fingers, 0);
        for (size_t i = 0; i < num_fingers; i++) {
            size_t num_permutations = factorial[num_fingers] / factorial[num_fingers - i - 1];
            permutations_vector.push_back(Matrix_MxN::Zero(num_permutations, i + 1));
        }
        size_t n = pow(2, num_fingers);
        for (size_t i = 1; i < n; i++) {
            std::bitset<num_fingers> indicator = std::bitset<num_fingers>(i);
            size_t index = indicator.count() - 1;
            std::vector<size_t> set;
            for (size_t j = 0; j < num_fingers; j++) {
                if (indicator[j]) {
                    set.push_back(j);
                }
            }
            do {
                for (size_t k = 0; k < set.size(); k++) {
                    permutations_vector[index](permutations_counts[index], k) = set[k];
                }
                permutations_counts[index]++;
            } while (std::next_permutation(set.begin(), set.end()));
        }
    }
	

    void initialize_directions_vector() {
        /*for (size_t i = 0; i < num_fingers; i++) {
            size_t num_directions = pow(2, i + 1);
            directions_vector.push_back(Matrix_MxN::Zero(num_directions, i + 1));
        }
        for (size_t num_directions = 0; num_directions < num_fingers; num_directions++) {
            size_t n = pow(2, num_directions + 1);
            for (size_t j = 0; j <= n; j++) {
                std::bitset<num_fingers> indicator = std::bitset<num_fingers>(j);
                for (size_t k = 0; k <= num_directions; k++) {
                    if (indicator[k]) {
                        directions_vector[num_directions](j, k) = 1;
                    }
                }
            }
        }*/


        for (size_t i = 0; i < num_fingers; i++) {
            directions_vector.push_back(Matrix_MxN::Zero(2, i + 1));
        }
        for (size_t num_directions = 0; num_directions < num_fingers; num_directions++)
            for (size_t k = 0; k < num_directions; k++) {
                directions_vector[num_directions](0, k) = 0;
                directions_vector[num_directions](1, k) = 1;
            }
    }


    /*size_t get_num_permutations(){
        return permutations_vector[num_targets - 2].rows();
    }*/
    size_t get_num_permutations(){
        return num_permutations;
    }
    VectorN get_permutation(size_t i){
        return permutations_vector[num_targets - 2].row(i);
    }
    size_t get_num_directions(){
        size_t num_direction_targets = num_directions;
        if (num_direction_targets == 0) return 0;
        return directions_vector[num_direction_targets - 1].rows();
    }
    VectorN get_direction(size_t i){
        return directions_vector[num_directions - 1].row(i);
    }
    size_t get_num_point_targets(){
        return point_targets_indices.size();
    }
    size_t get_num_line_targets(){
        return line_targets_indices.size();
    }
    size_t get_num_plane_targets(){
        return 2;
    }
    size_t get_num_direction_targets(){
        return direction_targets_indices.size();
    }
    std::vector<size_t> get_point_targets_indices(){
        return point_targets_indices;
    }
    std::vector<size_t> get_line_targets_indices(){
        return line_targets_indices;
    }
    std::vector<size_t> get_direction_targets_indices(){
        return direction_targets_indices;
    }
    Vector3 get_point_target(size_t i){
        return point_targets.row(i);
    }
    Vector3 get_line_target_start(size_t i){
        return line_targets.row(i).head(3);
    }
    Vector3 get_line_target_end(size_t i){
        return line_targets.row(i).tail(3);
    }
    Vector3 get_plane_target_origin(){
        return plane_targets.head(3);
    }
    Vector3 get_direction_target(size_t direction_number, size_t index, size_t i) {
        Matrix_MxN directions = directions_vector[num_directions - 1];
        size_t direction = directions(direction_number, i);
        if (direction == (size_t) 0)
            return direction_targets.row(index).head(3);
        /// TODO: anastasia is this correct?
        else /*(direction == (size_t) 1)*/
            return direction_targets.row(index).tail(3);
    }
    Vector3 get_line_target_normal(size_t i){
        Vector3 p1 = line_targets.row(i).head(3);
        Vector3 p2 = line_targets.row(i).tail(3);
        Vector3 normal = p2 - p1;
        normal = normal/normal.norm();
        return normal;
    }
    Vector3 get_plane_target_normal(){
        Vector3 p1 = plane_targets.head(3);
        Vector3 p2 = plane_targets.tail(3);
        Vector3 normal = p2 - p1;
        normal = normal/normal.norm();
        return normal;
    }
    Vector3 get_point_source(size_t id){
        if (id == joint_indices.palm.root) return get_palm_center();

        Joint * joint = skeleton->getJoint(id);
        Vector3 source = joint->getGlobalTranslation();
        return source;
    }
    Vector3 get_line_source(size_t id){
        Joint * joint = skeleton->getJoint(id);
        Vector3 source = joint->getGlobalTranslation();
        return source;
    }
    Vector3 get_plane_source(size_t j){
        Joint * joint = skeleton->getJoint(joint_indices.palm.root);
        Vector3 source = joint->getGlobalTranslation();
        Segment segment = cylinders->getSegments()[0];
        Mat3f T = joint->getGlobalRotation();
        Vector3 heights_axis = T * segment.length * up_direction;
        Vector3 width_axis = T * segment.radius1 * side_direction;
        Vector3 palm_center  = source + heights_axis / 2;
        switch (j) {
        /*case 0: // bottom left corner
            return palm_center - heights_axis / 2 - width_axis; break;
        case 1: // bottom right corner
            return palm_center - heights_axis / 2 + width_axis; break;
        case 2: // top left corner
            return palm_center + heights_axis / 2 - width_axis; break;
        case 3: // top right corner
            return palm_center + heights_axis / 2 + width_axis; break;
        }*/
        case 0: // bottom left corner
            return palm_center - width_axis; break;
        case 1: // bottom right corner
            return palm_center + width_axis; break;
        }
        
        /// TODO: anastasia, why is this necessary?
        return Vector3(0,0,0);
    }
    size_t get_plane_id(){
        return joint_indices.palm.root;
    }
    std::vector<size_t> get_ids(size_t permutation_number, size_t index){
        //cout << "[DetectionStream::get_ids()]" << endl;

        std::vector<size_t> ids;
        if (index == 0) {
            ids.push_back(joint_indices.palm.root);
            return ids;
        }
        Matrix_MxN permutations = permutations_vector[num_targets - 2];
        size_t finger = permutations(permutation_number, index - 1);
        //cout << "   source = " << finger << endl;

        switch (finger){
        case 0:
            ids.push_back(joint_indices.thumb.tip); ids.push_back(joint_indices.thumb.mid2); ids.push_back(joint_indices.thumb.mid1); break;
        case 1:
            ids.push_back(joint_indices.index.tip); ids.push_back(joint_indices.index.mid2); ids.push_back(joint_indices.index.mid1); break;
        case 2:
            ids.push_back(joint_indices.middle.tip); ids.push_back(joint_indices.middle.mid2); ids.push_back(joint_indices.middle.mid1); break;
        case 3:
            ids.push_back(joint_indices.ring.tip); ids.push_back(joint_indices.ring.mid2); ids.push_back(joint_indices.ring.mid1); break;
        case 4:
            ids.push_back(joint_indices.pinky.tip); ids.push_back(joint_indices.pinky.mid2); ids.push_back(joint_indices.pinky.mid1); break;
        }
        return ids;
    }
    size_t get_num_targets(){
        return num_targets;
    }
    Vector3 get_palm_center(){
        Joint * joint = skeleton->getJoint(joint_indices.palm.root);
        Vector3 source = joint->getGlobalTranslation();
        Segment segment = cylinders->getSegments()[0];
        Vector3 main_axis = segment.length * up_direction;
        Mat3f T = joint->getGlobalRotation();
        Vector3 shift = 0.5 * T * main_axis;
        Vector3 palm_center  = source + shift;
        return palm_center;
    }

    void set_point_targets(const Matrix_MxN & point_targets) {
        if (call_stack) cout << "[DetectionStream::set_point_targets()]" << endl;
        this->point_targets = point_targets;
    }
    void set_line_targets(const Matrix_MxN & line_targets) {
        if (call_stack) cout << "[DetectionStream::set_line_targets()]" << endl;
        this->line_targets = line_targets;
    }
    void set_plane_targets(const VectorN & plane_targets) {
        if (call_stack) cout << "[DetectionStream::set_plane_targets()]" << endl;
        this->plane_targets = plane_targets;
    }
    void set_direction_targets(const Matrix_MxN & direction_targets) {
        if (call_stack) cout << "[DetectionStream::set_direction_targets()]" << endl;
        this->direction_targets = direction_targets;
    }
    void set_point_targets_indices(const std::vector<size_t> & point_targets_indices) {
        if (call_stack) cout << "[DetectionStream::set_point_targets_indices()]" << endl;
        this->point_targets_indices = point_targets_indices;
    }
    void set_line_targets_indices(const std::vector<size_t> & line_targets_indices) {
        if (call_stack) cout << "[DetectionStream::set_line_targets_indices()]" << endl;
        this->line_targets_indices = line_targets_indices;
    }
    void set_direction_targets_indices(const std::vector<size_t> & direction_targets_indices) {
        if (call_stack) cout << "[DetectionStream::set_direction_targets_indices()]" << endl;
        this->direction_targets_indices = direction_targets_indices;
    }
    void set_num_targets(size_t num_targets){
        if (call_stack) cout << "[DetectionStream::set_num_targets()]" << endl;
        this->num_targets = num_targets;
    }
    void set_num_directions(size_t num_directions){
        if (call_stack) cout << "[DetectionStream::set_num_directions()]" << endl;
        this->num_directions = num_directions;
    }
    void set_radial_order(const std::vector<size_t> & radial_order) {
        if (call_stack) cout << "[DetectionStream::set_radial_order()]" << endl;
        this->radial_order = radial_order;
    }
    void set_linear_order(const std::vector<size_t> & linear_order) {
        if (call_stack) cout << "[DetectionStream::set_linear_order()]" << endl;
        this->linear_order = linear_order;
    }
    void set_thumb_index(int thumb_index){
        this->thumb_index = thumb_index;
    }
    int get_thumb_index(){
        return this->thumb_index;
    }
    void set_hand_side(int hand_side){
        this->hand_side = hand_side;
    }

    void update_permutations_vector() {
        if (call_stack) cout << "[DetectionStream::update_permutations_vector()]" << endl;
        std::vector<std::pair<size_t, size_t>> indices = std::vector<std::pair<size_t, size_t>>(num_fingers, std::pair<size_t, size_t>(1, 1));

        size_t index = num_fingers - 1;
        size_t count = 0;

        if (hand_side == -1 || hand_side == 1) {
            for (size_t i = 0; i < num_fingers; i++) {
                for (size_t k = 0; k < num_fingers; k++)
                    indices[k] = std::pair<size_t, size_t>(radial_order[(i + k) % num_fingers], k);
                std::sort(indices.begin(), indices.end(), size_t_comparator);
                if (thumb_index == -1 || indices[thumb_index].second == 0) {
                    for (size_t k = 0; k < num_fingers; k++)
                        permutations_vector[index](count, k) = indices[k].second;
                    count++;
                }
            }
        }
        if (hand_side == -1 || hand_side == 0) {
            for (size_t i = 0; i < num_fingers; i++) {
                for (size_t k = 0; k < num_fingers; k++)
                    indices[k] = std::pair<size_t, size_t>(radial_order[(i + (num_fingers - k - 1)) % num_fingers], k);
                std::sort(indices.begin(), indices.end(), size_t_comparator);
                if (thumb_index == -1 || indices[thumb_index].second == 0) {
                    for (size_t k = 0; k < num_fingers; k++)
                        permutations_vector[index](count, k) = indices[k].second;
                    count++;
                }
            }
        }
        if (hand_side == -1 || hand_side == 1) {
            for (size_t k = 0; k < num_fingers; k++)
                indices[k] = std::pair<size_t, size_t>(linear_order[k], k);
            std::sort(indices.begin(), indices.end(), size_t_comparator);
            if (thumb_index == -1 || indices[thumb_index].second == 0) {
                for (size_t k = 0; k < num_fingers; k++)
                    permutations_vector[index](count, k) = indices[k].second;
                count++;
            }
        }
        if (hand_side == -1 || hand_side == 0) {
            for (size_t k = 0; k < num_fingers; k++)
                indices[k] = std::pair<size_t, size_t>(linear_order[num_fingers - k - 1], k);
            std::sort(indices.begin(), indices.end(), size_t_comparator);
            if (thumb_index == -1 || indices[thumb_index].second == 0) {
                for (size_t k = 0; k < num_fingers; k++)
                    permutations_vector[index](count, k) = indices[k].second;
                count++;
            }
        }
        num_permutations = count;
    }

    void display_current_constraints(){
        cout << "plane_target = " << endl << plane_targets << endl << endl;
        cout << "point_target = " << endl << point_targets << endl << endl;
        cout << "line_target = " << endl << line_targets << endl << endl;
        cout << "direction_target = " << endl << direction_targets << endl << endl;
        cout << "num_directions = " << endl << num_directions << endl << endl;
        cout << "num_targets = " << endl << num_targets << endl << endl;
        cout << "point_target_indices = " << endl;
        for (int k = 0; k < point_targets_indices.size(); ++k) {
            cout << point_targets_indices[k] << " ";
        } cout << endl << endl;
        cout << "line_targets_indices = " << endl;
        for (int k = 0; k < line_targets_indices.size(); ++k) {
            cout << line_targets_indices[k] << " ";
        } cout << endl << endl;
        cout << "direction_targets_indices = " << endl;
        for (int k = 0; k < direction_targets_indices.size(); ++k) {
            cout << direction_targets_indices[k] << " ";
        } cout << endl << endl;
    }
};
