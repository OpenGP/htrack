#include "Temporal.h"
#include "tracker/Data/DataFrame.h"
#include "tracker/DataStructure/SkeletonSerializer.h"
#include "tracker/TwSettings.h"

/// Queue for temporal coherence
class SolutionQueue{
public:
   typedef std::vector<Scalar> Solution;
   typedef std::map<int, Solution> Solutions;
   Solutions solutions;

   bool valid(int id){
       return id >= 2
           && solutions.find(id-1) != solutions.end()
           && solutions.find(id-2) != solutions.end();
   }
   void set(int id, const Solution &s){
       solutions[id] = s;
   }
   void update(int id, const Solution &s){
       set(id, s);
       for(auto it = solutions.begin(); it != solutions.end();){
           int d = id - it->first;
           if(d > 2 || d < 0){
               it = solutions.erase(it);
           } else {
               ++it;
           }
       }
   }
};

namespace energy{

void Temporal::init(SkeletonSerializer *skeleton){
    this->skeleton = skeleton;
    this->solution_queue = new SolutionQueue();
    tw_settings->tw_add(settings->temporal_coherence1_enable, "1st Order",  "group=Temporal");
    tw_settings->tw_add(settings->temporal_coherence2_enable, "2nd Order",   "group=Temporal");
    tw_settings->tw_add(settings->temporal_coherence1_weight, "weight(1st)", "group=Temporal");
    tw_settings->tw_add(settings->temporal_coherence2_weight, "weight(2nd)", "group=Temporal");
}

Temporal::~Temporal(){
    if(solution_queue) delete solution_queue;
}

void Temporal::track(LinearSystem &system, DataFrame &frame){
    track(system, frame.id, true); ///< 1st order
    track(system, frame.id, false); ///< 2nd order
    /// make sure you don't break stuff
    if(Energy::safety_check)
        Energy::has_nan(system);
}

void Temporal::update(int frame_id, const std::vector<Scalar>& theta){
    solution_queue->update(frame_id, theta);
}

void extract_positions(
        Skeleton*const skeleton,
        const vector<int> &joint_ids,
        const vector<Scalar> &thetas,
        vector<Vector3> &positions)
{
    if(thetas.empty()) return;

    vector<float> current = skeleton->getCurrentParameters();
    positions.resize(joint_ids.size());

    skeleton->set(thetas);

    for(size_t i = 0; i < joint_ids.size(); ++i){
        Joint *joint = skeleton->getJoint(joint_ids[i]);
        positions[i] = joint->getGlobalTranslation();
    }

    skeleton->set(current);
}

void Temporal::temporal_coherence_init()
{
    joint_ids.clear();
    pos_prev1.clear();
    pos_prev2.clear();

    vector<Joint *> &joints = skeleton->getJoints_();

    for(Joint *joint : joints){
        if(!skeleton->isStructural(joint)){
            joint_ids.push_back(skeleton->getID(joint->getName()));
            pos_prev1.push_back(joint->getGlobalTranslation());
            pos_prev2.push_back(joint->getGlobalTranslation());

            if(joint->hasChildren()){
                Joint *child = joint->getChildren()[0];
                joint_ids.push_back(skeleton->getID(child->getName()));
                pos_prev1.push_back(child->getGlobalTranslation());
                pos_prev2.push_back(child->getGlobalTranslation());
            }
        }
    }
}

void Temporal::track(LinearSystem& system, int fid, bool first_order)
{
    if(first_order){
        if(!settings->temporal_coherence1_enable) return;
    } else {
        if(!settings->temporal_coherence2_enable) return;
    }

    // TIMED_BLOCK(timer,"Worker::temporal_coherence_track(extract positions)")
    {
        if(joint_ids.empty()){
            temporal_coherence_init();
        }

        if(solution_queue->valid(fid)){
            if(fid != fid_curr){
                extract_positions(skeleton, joint_ids, solution_queue->solutions[fid-1], pos_prev1);
                extract_positions(skeleton, joint_ids, solution_queue->solutions[fid-2], pos_prev2);
                fid_curr = fid;
            }
        } else {
            // solution queue invalid, do nothing
            return;
        }
    }

    // TIMED_BLOCK(timer,"Worker::temporal_coherence_track(compute jacobian)")
    {
        Matrix_MxN J = Matrix_MxN::Zero(3*joint_ids.size(), num_thetas);
        VectorN e = VectorN::Zero(3*joint_ids.size());

        for(size_t i = 0; i < joint_ids.size(); ++i){
            Joint *joint = skeleton->getJoint(joint_ids[i]);
            int id = skeleton->getID(joint->getName());
            Vector3 pos_frame = joint->getGlobalTranslation();

            J.block(3*i,0,3,num_thetas) = skeleton->jacobian(id, pos_frame);
            if(first_order){
                e.block(3*i,0,3,1) = pos_prev1[i] - pos_frame;
            } else {
                e.block(3*i,0,3,1) = 2*pos_prev1[i] - pos_prev2[i] - pos_frame;
            }
        }

        Scalar omega = 1.0f;
        if(first_order) {
            omega = settings->temporal_coherence1_weight;
        } else {
            omega = settings->temporal_coherence2_weight;
        }

        Matrix_MxN JT = J.transpose(); // transp(J) is inefficient for small matrices
        system.lhs += omega * JT * J;
        system.rhs += omega * JT * e;
    }
    
    ///--- Check
    if(Energy::safety_check)
        Energy::has_nan(system);
}

}
