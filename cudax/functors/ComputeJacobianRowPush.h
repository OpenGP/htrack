#pragma once
#include "ComputeJacobianRow.h"

//== NAMESPACE ================================================================
namespace cudax {
//=============================================================================

class ComputeJacobianRowPush : public ComputeJacobianRow{
private:
    ///--- These are for the extra_push
    int* sensor_dtform_idxs;
    float weight;
public:
    ComputeJacobianRowPush(J_row* J_raw, float* e_raw) : ComputeJacobianRow(J_raw, e_raw){
        this->sensor_dtform_idxs = thrust::raw_pointer_cast(::sensor_dtform_idxs->data());
        weight = settings->fit2D_weight;
        
        assert(cudax::render_color!=NULL);
        assert(render_points!=NULL);
    }
        
public:
    __device__
    void operator()(MeshGrid::Elem& off){
        if( type(off) != PixelType::CONSTRAINT_EXTRA_PUSH) return;
                
        int row_index = constraint_index(off);
        J_row* J_sub = J_raw + row_index;
        float* e_sub = e_raw + row_index;
        
        int joint_id = (int) tex2D(color_tex, off.x, off.y);
        
        ///--- Fetch closest point on sensor data
        int closest_idx = sensor_dtform_idxs[off.z];
        int row = closest_idx / width;
        int col = closest_idx - width*row;
        glm::vec2 p_rend(off.x, off.y);
        glm::vec2 p_sens(col, row);
        glm::vec2 p_diff = p_sens-p_rend;

//#define PAD_DISTANCE_TRANSFORM
#ifdef PAD_DISTANCE_TRANSFORM
        float threshold = 3.0f;
        float l_orig = glm::length(p_diff);
        if(l_orig < threshold)
            p_diff = glm::vec2(0);
        else
            p_diff *= (l_orig - threshold) / l_orig;
#endif
        
        /// Fills LHS        
        glm::vec3 p_rend_3D;
        p_rend_3D[0] = tex2D(extra_tex, off.x, off.y).x;
        p_rend_3D[1] = tex2D(extra_tex, off.x, off.y).y;
        p_rend_3D[2] = tex2D(extra_tex, off.x, off.y).z;
                
        glm::mat3x2 J_proj = projection_jacobian(p_rend_3D);
        // skeleton_jacobian(joint_id, p_rend_3D, J_sub, projector, true);
        
// #define BIAS_PALM_SILHO
#ifdef BIAS_PALM_SILHO
        int weight = this->weight;
        if(joint_id==3)
            weight = this->weight/**10*/;
        else
            weight = 0; //this->weight;
#endif
        
        ///--- Compute LHS
        for(int i_column=0; i_column<CHAIN_MAX_LENGTH; i_column++){
            int jointinfo_id = chains[joint_id].data[i_column];
            if(jointinfo_id==-1) break;
            const CustomJointInfo& jinfo = jointinfos[jointinfo_id];
            glm::vec3& axis = jointinfos[jointinfo_id].axis;
            
//#define DISABLE_PUSH_ABDUCTION
#ifdef DISABLE_PUSH_ABDUCTION
            if(jointinfo_id==13) continue;
            if(jointinfo_id==17) continue;
            if(jointinfo_id==21) continue;
            if(jointinfo_id==25) continue;
#endif
            
            switch(jinfo.type){
                case 1:
                {
                    glm::vec3 col = glm::vec3( jointinfos[jointinfo_id].mat * glm::vec4( axis, 1 ) );
                    glm::vec2 jcol = J_proj * col;
                    (J_sub+0)->data[jinfo.index] = weight * jcol.x;
                    (J_sub+1)->data[jinfo.index] = weight * jcol.y;
                    break;
                }
                case 0: // ROT
                {
                    glm::vec3 t(jointinfos[jointinfo_id].mat[3][0],jointinfos[jointinfo_id].mat[3][1],jointinfos[jointinfo_id].mat[3][2]);
                    glm::vec3 a = glm::normalize(glm::vec3(jointinfos[jointinfo_id].mat * glm::vec4( axis, 1 )) - t);
                    glm::vec3 col = glm::cross(a, p_rend_3D - t);
                    glm::vec2 jcol = J_proj * col;
                    (J_sub+0)->data[jinfo.index] = weight * jcol.x;
                    (J_sub+1)->data[jinfo.index] = weight * jcol.y;                    
                    break;
                }
            }
        }
        
        /// Fills RHS
        *(e_sub+0) = weight * p_diff.x;
        *(e_sub+1) = weight * p_diff.y;
    }
};

//=============================================================================
} // namespace cudax
//=============================================================================
