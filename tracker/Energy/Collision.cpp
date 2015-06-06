#include "Collision.h"

#include "DebugRenderer.h"
#include "util/mylogger.h"

#include "tracker/Legacy/geometry/Cylinders.h"

#include "tracker/Worker.h"
#include "tracker/TwSettings.h"
#include "tracker/Data/DataFrame.h"
#include "tracker/DataStructure/CustomJointInfo.h"
#include "tracker/DataStructure/SkeletonSerializer.h"

namespace geometry{

/* Copyright 2001 softSurfer, 2012 Dan Sunday
   This code may be freely used and modified for any purpose
   providing that this copyright notice is included with it.
   SoftSurfer makes no warranty for this code, and cannot be held
   liable for any real or imagined damage resulting from its use.
   Users of this code must verify correctness for their application.
*/
 std::pair<Vector3, Vector3>  segment_to_segment_distance( std::pair<Vector3, Vector3> S1, std::pair<Vector3, Vector3> S2, bool display){
    double epsilon = 0.00000001;
    Vector3   u = S1.second - S1.first;
    Vector3   v = S2.second - S2.first;
    Vector3   w = S1.first - S2.first;
    float a = u.dot(u);       // always >= 0
    float b = u.dot(v);
    float c = v.dot(v);         // always >= 0
    float d = u.dot(w);
    float e = v.dot(w);
    float D = a*c - b*b;        // always >= 0
    float sc, sN, sD = D;       // sc = sN / sD, default sD = D >= 0
    float tc, tN, tD = D;       // tc = tN / tD, default tD = D >= 0

    // compute the line parameters of the two closest points
    if (D < epsilon) { // the lines are almost parallel
        sN = 0.0;         // force using point P0 on segment S1
        sD = 1.0;         // to prevent possible division by 0.0 later
        tN = e;
        tD = c;
    }
    else {                 // get the closest points on the infinite lines
        sN = (b*e - c*d);
        tN = (a*e - b*d);
        if (sN < 0.0) {        // sc < 0 => the s=0 edge is visible
            sN = 0.0;
            tN = e;
            tD = c;
        }
        else if (sN > sD) {  // sc > 1  => the s=1 edge is visible
            sN = sD;
            tN = e + b;
            tD = c;
        }
    }

    if (tN < 0.0) {            // tc < 0 => the t=0 edge is visible
        tN = 0.0;
        // recompute sc for this edge
        if (-d < 0.0)
            sN = 0.0;
        else if (-d > a)
            sN = sD;
        else {
            sN = -d;
            sD = a;
        }
    }
    else if (tN > tD) {      // tc > 1  => the t=1 edge is visible
        tN = tD;
        // recompute sc for this edge
        if ((-d + b) < 0.0)
            sN = 0;
        else if ((-d + b) > a)
            sN = sD;
        else {
            sN = (-d +  b);
            sD = a;
        }
    }
    // finally do the division to get sc and tc
    sc = (abs(sN) < epsilon ? 0.0 : sN / sD);
    tc = (abs(tN) < epsilon ? 0.0 : tN / tD);

    // get the difference of the two closest points
    Vector3 dP = w + (sc * u) - (tc * v);  // =  S1(sc) - S2(tc)

    Vector3 P1 = S1.first + sc * u;
    Vector3 P2 = S2.first + tc * v;
    std::pair<Vector3, Vector3> closest_distance = std::pair<Vector3, Vector3>(P1, P2);

#if 0
    if (display == true){
        std::vector<Vector3> points;
        std::vector<std::pair<Vector3, Vector3>> segments;
        points.push_back(S1.first);
        points.push_back(S1.second);
        points.push_back(S2.first);
        points.push_back(S2.second);
        points.push_back(P1);
        points.push_back(P2);
        segments.push_back(S1);
        segments.push_back(S2);
        segments.push_back(std::pair<Vector3, Vector3>(P1, P2));
        DebugRenderer::instance().add_points(points, Vector3(1, 0, 0));
        DebugRenderer::instance().add_segments(segments, Vector3(0, 0, 1));
        worker->updateGL();
    }
#endif

    //return dP.norm();
    return closest_distance;
}

bool ray_triangle_intersection(Vector3 O, Vector3 D, Vector3 P1, Vector3 P2, Vector3 P3, bool display){
    double epsilon = 0.00000001;

#if 0
    if (display == true){
        std::vector<Vector3> points;
        std::vector<std::pair<Vector3, Vector3>> segments;
        points.push_back(P1);
        points.push_back(P2);
        points.push_back(P3);
        points.push_back(O);
        float offset = 1000;
        segments.push_back(std::pair<Vector3, Vector3>(P1, P2));
        segments.push_back(std::pair<Vector3, Vector3>(P2, P3));
        segments.push_back(std::pair<Vector3, Vector3>(P1, P3));
        segments.push_back(std::pair<Vector3, Vector3>(O, offset * D));
        DebugRenderer::instance().add_points(points, Vector3(1, 0, 0));
        DebugRenderer::instance().add_segments(segments, Vector3(0, 0, 1));
        worker->updateGL();
    }
#endif

    Vector3 e1 = P2 - P1;
    Vector3 e2 = P3 - P1;
    Vector3 q  = D.cross(e2);
    float a  = e1.dot(q); // determinant of the matrix M

    if (a > -epsilon && a < epsilon) {
        LOG(INFO) << "The vector is parallel to the plane (the intersection is at infinity).";
        return false;
    }

    float f = 1/a;
    Vector3 s = O - P1;
    float u = f * s.dot(q);

    if (u < 0.0){
        LOG(INFO) << "The intersection is outside of the triangle, case 1.";
        return false;
    }

    Vector3 r = s.cross(e1);
    float v = f * D.dot(r);

    if (v < 0.0 || u + v > 1.0){
        LOG(INFO) << "The intersection is outside of the triangle, case 2.";
        return false;
    }

    float t = f * e2.dot(r);
    if (t < 0.0) {
        LOG(INFO) << "The intersection is in the continuation of the ray.";
        return false;
    }

#if 0
    if (display == true){
        std::vector<Vector3> points;
        points.push_back(O + t * D);
        DebugRenderer::instance().add_points(points, Vector3(1, 0, 0));
        worker->updateGL();
    }
#endif

    return true;
}

} ///< geometry::

// Apply transformation to the given point
Vector3 transform_point_position(Vector3 point, Mat4f transformation){
    VectorN homogen_point(4);
    homogen_point << point, 1.0;
    VectorN transformed_homogen_point = transformation * homogen_point;
    return (transformed_homogen_point/ transformed_homogen_point[3]).head(3);
}


// Return axis endpoint of the segment
std::pair<Vector3, Vector3> get_axis_endpoints(Segment segment){
    Joint * joint = segment.joint;
    Vector3 axis_start = Vector3(0, 0, 0);
    Vector3 axis_end = Vector3(0, segment.length, 0);
    Vector3 transformed_axis_start = transform_point_position(axis_start, joint->getGlobalTransformation());
    Vector3 transformed_axis_end = transform_point_position(axis_end, joint->getGlobalTransformation());
    return std::pair<Vector3, Vector3>(transformed_axis_start, transformed_axis_end);
}


// Display skeleton with segments showing fingers width
void debug_display_phalange(std::vector<Segment> segments) {
    DebugRenderer::instance().clear();
    std::vector<Vector3> hand_points;
    std::vector<Vector3> hand_colors;
    std::vector<std::pair<Vector3, Vector3>> hand_lines;


    Vector3 dark_green(0, 0.8, 0), green(0.3, 1, 0.3);

    for (int i = 0; i<segments.size(); i++){
        Segment segment = segments[i];
        Joint* joint = segment.joint;

        Vector3 initial_top = Vector3(0, segment.length, 0);
        Vector3 initial_bottom = Vector3(0, 0, 0);
        Vector3 top = transform_point_position(initial_top, joint->getGlobalTransformation());
        Vector3 bottom = transform_point_position(initial_bottom, joint->getGlobalTransformation());

        hand_lines.push_back(std::pair<Vector3, Vector3>(top, bottom));
        hand_colors.push_back(dark_green);

        // create very rough approximations of spheres on both ends of the capsule
        std::vector<Vector3> directions { Vector3(1, 0, 0), Vector3(-1, 0, 0), Vector3(0, 1, 0),
                    Vector3(0, -1, 0), Vector3(0, 0, 1), Vector3(0, 0, -1) };
        for (auto direction : directions){
            Vector3 top_point = segment.radius2 * direction + initial_top;
            top_point = transform_point_position(top_point, joint->getGlobalTransformation());
            hand_lines.push_back(std::pair<Vector3, Vector3>(top, top_point));
            hand_points.push_back(top_point);
            hand_colors.push_back(green);

            Vector3 bottom_point = segment.radius2 * direction + initial_bottom;
            bottom_point = transform_point_position(bottom_point, joint->getGlobalTransformation());
            hand_lines.push_back(std::pair<Vector3, Vector3>(bottom, bottom_point));
            hand_points.push_back(bottom_point);
            hand_colors.push_back(green);
        }
    }

    DebugRenderer::instance().add_points(hand_points,  Vector3(1, 1, 0));
    DebugRenderer::instance().add_segments(hand_lines, hand_colors);
    //::glarea->updateGL();
}


// Display collision constraints
void debug_display_constraints(std::pair<Vector3, Vector3> shortest_path, Vector3 current_intersection, Vector3 other_intersection){
    std::vector<std::pair<Vector3, Vector3>> lines;
    std::vector<Vector3> colors, points;
    lines.push_back(std::pair<Vector3, Vector3>(shortest_path.first, current_intersection));
    points.push_back(current_intersection);
    colors.push_back(Vector3(1, 0.5, 0));
    lines.push_back(std::pair<Vector3, Vector3>(shortest_path.second, other_intersection));
    points.push_back(other_intersection);
    colors.push_back(Vector3(0, 0.5, 1));
    DebugRenderer::instance().add_segments(lines, colors);
    DebugRenderer::instance().add_points(points, colors);
    //::glarea->updateGL();
}

// A(i, j) = 1, if i is adjacent to j
// A(i, j) = 2, if i should not be compared to j
// A(i, j) = 0, if we should test the joint i and j for collision
Matrix_MxN create_adjacency_matrix(Skeleton* skeleton,  std::vector<Segment> segments){
    Matrix_MxN adjacency_matrix = Matrix_MxN::Zero(segments.size(), segments.size());

    std::map<Joint*,int> mapper;
    for (int i = 0; i<segments.size(); i++){
        Joint* joint = segments[i].joint;
        mapper[joint] = i;
    }

    for (int i = 0; i<segments.size(); i++){
        Joint* joint = segments[i].joint;
        if (skeleton->isStructural(joint)) continue;

        Joint* parent  = joint->getParent();
        std::vector<Joint *> children = joint->getChildren();

        if (parent != NULL)
            adjacency_matrix(i, mapper[parent])  = 1;
        if (joint->hasChildren())
            for (auto child : children)
                adjacency_matrix(i, mapper[child])  = 1;
    }
    return adjacency_matrix;
}

std::vector<std::vector<std::pair<Vector3, Vector3>>> create_distance_matrix(Skeleton* skeleton, std::vector<Segment> segments,  Matrix_MxN adjacency_matrix){

    std::vector<std::vector<std::pair<Vector3, Vector3>>> distance_matrix;
    std::pair<Vector3, Vector3> big_distance(Vector3(0, 0, 0), Vector3(RAND_MAX, RAND_MAX, RAND_MAX));
    std::vector<std::pair<Vector3, Vector3>> current_row(segments.size(), big_distance);
    distance_matrix.push_back(current_row);
    for (int i = 1; i<segments.size(); i++){
        Segment current_segment = segments[i];
        Joint* current_joint = current_segment.joint;

        distance_matrix.push_back(current_row);

        if (skeleton->isStructural(current_joint)) continue;

        for (int j = 1; j < i; j++){
            Segment segment = segments[j];
            Joint* joint = segment.joint;
            if (skeleton->isStructural(joint)) continue;
            if (adjacency_matrix(i, j) != 0) continue;

            std::pair<Vector3, Vector3> shortest_path =
                    geometry::segment_to_segment_distance(get_axis_endpoints(current_segment),
                                                          get_axis_endpoints(segment), false);
            distance_matrix[i][j] = shortest_path;

        }
    }
    return distance_matrix;
}



namespace energy{

Matrix_3xN jacobian_block(Collision::Point point, SkeletonSerializer* skeleton){
    /*
     * CustomJointInfo {
     *  float mat[16] - homogenoius transformation matrix
     *  float axis[3] - axis to assemble jacobian column
     *  int type - type of joint axis
     *  int index - index of jacobian column
     * }
     */

    Vector3 s = point.p;
    size_t id = point.id;
    Matrix_3xN J = Matrix_3xN::Zero(3, skeleton->getMapping().getNumberOfParameters());

    const KinematicChain& kinematic_chain = skeleton->getKinematicChain();
    const JointTransformations& joint_transformations = skeleton->getJointTransformations();

    /*(for (int i = 0; i < kinematic_chain.size(); ++i) {
        ChainElement chain_element = kinematic_chain[i];
        for (int j = 0; j < CHAIN_MAX_LENGTH; ++j) {
            if (chain_element.data[j] == -1) break;
            cout << chain_element.data[j] << " ";
        } cout << endl;
    } cout << endl;*/
    /*for (int i = 0; i < joint_transformations.size(); ++i) {
        CustomJointInfo custom_joint_info = joint_transformations[i];
        cout << custom_joint_info << endl;
    }*/

    for(int i = 0; i < CHAIN_MAX_LENGTH; i ++){
        int ancestor_id = kinematic_chain[id].data[i];
        if(ancestor_id == -1) break;

        const CustomJointInfo& joint_info = joint_transformations[ancestor_id];
        Vector3 u = Eigen::Map<const Vector3>(joint_info.axis);
        Vector3 p = Vector3(joint_info.mat[12], joint_info.mat[13], joint_info.mat[14]);
        Transform3f T = Transform3f( Eigen::Map<const Matrix4>(joint_info.mat));
        Vector3 v = T * u - p;

        switch(joint_info.type){
        case TRANSLATION_AXIS:{
            J.col(joint_info.index) = v;
            break;
        }

        case ROTATION_AXIS:{
            J.col(ancestor_id) = v.cross(s - p);
            break;
        }
        }
    }
    return J;
}

void Collision::init(Worker *worker){
    this->worker = worker;
    this->skeleton = worker->skeleton;
    this->cylinders = worker->cylinders;
}

Scalar Collision::create_collision_constraints(){
    int discard_palm = 1;
    Scalar E = 0;

    // TIMED_SCOPE(timer, "Worker::create_collision_constraints");
    s.clear();
    t.clear();

    std::vector<Segment> segments = cylinders->getSegments();
    Matrix_MxN adjacency_matrix = create_adjacency_matrix(skeleton, segments);
    //debug_display_phalange(segments);

    std::vector<std::vector<std::pair<Vector3, Vector3>>> distance_matrix = create_distance_matrix(skeleton, segments, adjacency_matrix);
    for (int i = discard_palm; i<segments.size(); i++){
        Segment segment_i = segments[i];
        size_t id_i = skeleton->getID(segment_i.joint->getName());
        // cout << "name = " << segment_i.joint->getName() << ", id = " << id_i << endl;

        for (int j = discard_palm; j < i; j++){
            Segment segment_j = segments[j];
            size_t id_j = skeleton->getID(segment_j.joint->getName());

            std::pair<Vector3, Vector3> shortest_path = distance_matrix[i][j];
            float distance = (shortest_path.first - shortest_path.second).norm();

            /*Vector3 normal = (shortest_path.second - shortest_path.first) / distance;
            Vector3 surface_i = shortest_path.first + normal * segment_i.radius2;
            Vector3 surface_j = shortest_path.second - normal * segment_j.radius2;
            debug_display_constraints(shortest_path, surface_i, surface_j);*/


            if (distance > FLT_MIN && distance < (segment_i.radius2 + segment_j.radius2)){ //TODO: This is a bit weird that distance is sometime 0

                Vector3 normal = (shortest_path.second - shortest_path.first) / distance;
                Vector3 surface_i = shortest_path.first + normal * segment_i.radius2;
                Vector3 surface_j = shortest_path.second - normal * segment_j.radius2;

                s.push_back(Point(surface_i, id_i));
                t.push_back(Point(surface_j, id_j));
                n.push_back(normal);

                Scalar e = normal.transpose() * (surface_i - surface_j);
                E = E + e * e;
                //debug_display_constraints(shortest_path, surface_i, surface_j);
            }
        }
    }
    return E;
}

void Collision::collision_compute_update(LinearSystem &system, Scalar omega){
    // TIMED_SCOPE(timer,"Worker::collision_compute_update");
    const int d = skeleton->getMapping().getDimension();
    const int k = max(s.size(), t.size());

    Scalar fraction = settings->collision_fraction;
    Matrix_MxN J; VectorN rhs;

    if (settings->collision_new == false) {
        Scalar c = 2;
        Scalar fraction = settings->collision_fraction;
        J = Matrix_MxN::Zero(c * k, d);
        rhs = VectorN::Zero(c * k);
        for(int i = 0; i < k; ++i) {
            J.block(c * i, 0, 1, d) = n[i].transpose() * jacobian_block(s[i], skeleton);
            rhs.block(c * i, 0, 1, 1) = fraction * n[i].transpose() * (t[i].p - s[i].p);
            J.block(c * i + 1, 0, 1, d) = n[i].transpose() * jacobian_block(t[i], skeleton);
            rhs.block(c * i + 1, 0, 1, 1) = fraction *  n[i].transpose() * (s[i].p - t[i].p);
        }
        J.block(0, 0, c * k, num_thetas_rigid_motion) = Matrix_MxN::Zero(c * k, num_thetas_rigid_motion);
    }
    else {
        J = Matrix_MxN::Zero(k, d);
        rhs = VectorN::Zero(k);
        for(int i = 0; i < k; ++i) {
            J.block(i, 0, 1, d) = n[i].transpose() * jacobian_block(s[i], skeleton) - n[i].transpose() * jacobian_block(t[i], skeleton);
            rhs(i) = - fraction * (t[i].p - s[i].p).norm();
        }

        J.block(0, 0, k, num_thetas_rigid_motion) = Matrix_MxN::Zero(k, num_thetas_rigid_motion);
    }

    system.lhs += omega * J.transpose() * J;
    system.rhs += omega * J.transpose() * rhs;
}

void Collision::track(LinearSystem& system){
    // TIMED_SCOPE(timer,"Worker::collision_track");
    if(!settings->collision_enable) return;
    create_collision_constraints();
    collision_compute_update(system, settings->collision_weight);
    // cout << "E_collision = " << omega * E << endl;
}

} ///< energy::
