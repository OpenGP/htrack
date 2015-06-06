#include "cudax/kernel.h"
#include "cudax/MeshGrid.h"
#include "cudax/PixelIndexer.h"
#include "ClosestPoint.h"

//== NAMESPACE ================================================================
namespace cudax {
//=============================================================================

/// Computes the content of a row of the jacobian
struct ComputeJacobianRowExtraClosedForm : public ComputeJacobianRow{
    const glm::mat3x3& iproj; ///< to compute cloud from depth
    ClosestPoint matcher;

    bool debug_store;
    float *debug_correspondences_raw;

    float weight;
    bool point_to_plane;
    bool reweight;

public:
    ComputeJacobianRowExtraClosedForm(J_row* J_raw, float* e_raw, bool reweight) : 
        ComputeJacobianRow(J_raw, e_raw),         
        iproj( cudax::camera_matrix->D_inv_proj_matrix() )
    {
        point_to_plane = settings->fit3D_point2plane;
        weight = settings->fit3D_weight;
        this->reweight = reweight;
        debug_store = false;
        debug_correspondences_raw = NULL;
    }

    void store_data(float *debug_correspondences)
    {
        if(debug_correspondences != NULL){
            debug_store = true;
            debug_correspondences_raw = debug_correspondences;
        } else {
            debug_store = false;
            debug_correspondences_raw = NULL;
        }
    }

    ///--- Computes 3D point
    inline __device__ float4 sensor_point_at(const MeshGrid::Elem off){ 
        float depth = (float) SENSOR_DEPTH(off);
        glm::vec3 wrld = iproj * glm::vec3( off.x*depth, off.y*depth, depth);
        return make_float4(wrld[0], wrld[1], wrld[2], 0 /*unused*/);
    }

    // dot product
    inline __device__ float dot(float2 a, float2 b)
    {
        return a.x * b.x + a.y * b.y;
    }
    inline __device__ float2 difference(float2 a, float2 b)
    {
        return make_float2(a.x - b.x, a.y - b.y);
    }
    
    inline __device__ float4 difference(float4 a, float4 b)
    {
        return make_float4(a.x - b.x, a.y - b.y, a.z - b.z, 0);
    }
    inline __device__ float norm(float4 a)
    {
        return sqrt(a.x*a.x + a.y*a.y + a.z*a.z);
    }

    __device__
    float SAFE_DOT(glm::vec3 u, glm::vec3 v){
        return u.x*v.x + u.y*v.y + u.z*v.z;
    }

    /// Last argument accesses THREE rows!
    __device__
    void skeleton_jacobian(const int joint_id, const glm::vec3& pos, J_row* sub_J, glm::vec3 nrm=glm::vec3(0), bool project=false){
        for(int i_column=0; i_column<CHAIN_MAX_LENGTH; i_column++){
            // printf("i_column => jointinfo_id: %d %d\n", i_column, jointinfo_id);
            int jointinfo_id = chains[joint_id].data[i_column];
            if(jointinfo_id==-1) break;
            const CustomJointInfo& jinfo = jointinfos[jointinfo_id];
            glm::vec3& axis = jointinfos[jointinfo_id].axis;

            glm::vec3 col;
            switch(jinfo.type){
                case 1 /*TRA*/:
                {
                    col = glm::vec3( jointinfos[jointinfo_id].mat * glm::vec4( axis, 1 ) );
                    break;
                }
                case 0 /*ROT*/:
                {
                    glm::vec3 t(jointinfos[jointinfo_id].mat[3][0],jointinfos[jointinfo_id].mat[3][1],jointinfos[jointinfo_id].mat[3][2]);
                    glm::vec3 a = glm::normalize(glm::vec3(jointinfos[jointinfo_id].mat * glm::vec4( axis, 1 )) - t);
                    col = glm::cross(a, pos - t);
                    break;
                }
            }

            if(point_to_plane || project){
                sub_J->data[jinfo.index] = weight * SAFE_DOT( col, nrm );
            } else {
                (sub_J+0)->data[jinfo.index] = weight * col[0];
                (sub_J+1)->data[jinfo.index] = weight * col[1];
                (sub_J+2)->data[jinfo.index] = weight * col[2];
            }
        }
    }

    __device__
    void operator()(MeshGrid::Elem& off){
        if( type(off) != PixelType::CONSTRAINT_EXTRA_PULL) return;
        // printf("processing pixel[%d %d - %d] \n ", off.x, off.y, off.z);

        ///--- Access to a 3xN block of Jacobian
        // printf("  constraint_index(off): %d\n", constraint_index(off));
        J_row* J_sub = J_raw + 3*constraint_index(off);
        float* e_sub = e_raw + 3*constraint_index(off);

        float4 p_sensor = sensor_point_at(off);

        float4 p_model, n_model;
        int joint_id;
        matcher.closest_point(p_sensor, p_model, n_model, joint_id);
        if(!matcher.is_valid(p_sensor, p_model, n_model, joint_id)) return;

        // Project sensor point onto the line along the model point's normal.
        // This relocates the target point such that the corresponding source
        // point is its orthogonal projection onto the model, which stabilizes
        // the solution at the expense of slightly wrong target positions.
        // (Note: this is only relevant for palm segment constraints!)
        matcher.relocate_target(p_sensor, p_model, n_model);

        if(debug_store){
#if 0
            debug_correspondences_raw[6 * constraint_index(off) + 0] = p_model.x + 10*n_model.x;
            debug_correspondences_raw[6 * constraint_index(off) + 1] = p_model.y + 10*n_model.y;
            debug_correspondences_raw[6 * constraint_index(off) + 2] = p_model.z + 10*n_model.z;
#else
            debug_correspondences_raw[6 * constraint_index(off) + 0] = p_sensor.x;
            debug_correspondences_raw[6 * constraint_index(off) + 1] = p_sensor.y;
            debug_correspondences_raw[6 * constraint_index(off) + 2] = p_sensor.z;
#endif
            debug_correspondences_raw[6 * constraint_index(off) + 3] = p_model.x;
            debug_correspondences_raw[6 * constraint_index(off) + 4] = p_model.y;
            debug_correspondences_raw[6 * constraint_index(off) + 5] = p_model.z;
        }

        //printf("  p_model: %f %f %f %f\n", p_model.x, p_model.y, p_model.z, p_model.w);
        //printf("  joint_id: %d\n", joint_id);

        if(reweight){
            float d = norm(difference(p_sensor, p_model));
            //float w = sqrt(1.0f / (d + 1e-3));
            float w = rsqrt(d + 1e-3);
            if(d > 1e-3) weight *= w * 3.5f;
            // factor 3.5 compensates for residual magnitude change
        }
        
        if(point_to_plane){
            ///--- Fills LHS
            float4 n = n_model;
            //float4 n = SENSOR_NORMALS(off);
            //if( isnan(n.x) || isnan(n.y) || isnan(n.z) ) printf("ERROR: %f %f %f %f\n",n.x, n.y, n.z, n.w);
            glm::vec3 normal(n.x, n.y, n.z);
            skeleton_jacobian(joint_id, glm::vec3(p_model.x, p_model.y, p_model.z), J_sub, normal);
            ///--- Fills RHS
            glm::vec3 residual(p_sensor.x-p_model.x, p_sensor.y-p_model.y, p_sensor.z-p_model.z);
            *e_sub = weight * SAFE_DOT( residual, normal );
        } else {
            skeleton_jacobian(joint_id, glm::vec3(p_model.x, p_model.y, p_model.z), J_sub);
            e_sub[0] = weight * ( p_sensor.x - p_model.x );
            e_sub[1] = weight * ( p_sensor.y - p_model.y );
            e_sub[2] = weight * ( p_sensor.z - p_model.z );
        }
    }
};

//=============================================================================
} // namespace cudax
//=============================================================================
