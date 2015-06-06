#pragma once
#include <thrust/device_vector.h>
#include "cudax/cuda_glm.h"
#include "cudax/kernel.h"
#include "CustomJointInfo.h"

//== NAMESPACE ================================================================
namespace cudax {
//=============================================================================

struct SkeletonJacobian{
    CustomJointInfo* jointinfos; ///< device
    ChainElement* chains; ///< device
    
public:    
    SkeletonJacobian(CustomJointInfo* jointinfos, ChainElement* chains){
        assert(jointinfos!=NULL);
        assert(chains!=NULL);
        this->jointinfos = jointinfos;
        this->chains = chains;
    }

    /// Last argument accesses THREE rows!
    __device__
    void operator()(const int joint_id, const glm::vec3& pos, J_row* sub_J){
        J_row& J0 = *(sub_J+0);
        J_row& J1 = *(sub_J+1);
        J_row& J2 = *(sub_J+2);

//        printf("joint_id: %d\n", joint_id);
        for(int i_column=0; i_column<CHAIN_MAX_LENGTH; i_column++){
            printf("i_column => jointinfo_id: %d %d\n", chains[joint_id]);
            
#if 0       
            int jointinfo_id = chains[joint_id].data[i_column];
            printf("i_column => jointinfo_id: %d %d\n", i_column, jointinfo_id);
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
                    glm::vec3 a = glm::vec3(jointinfos[jointinfo_id].mat * glm::vec4( axis, 1 )) - t;
                    col = glm::cross(a, pos - t);                    
                    break;
                }
            }
            J0.data[jinfo.index] = col[0];
            J1.data[jinfo.index] = col[1];
            J2.data[jinfo.index] = col[2];
#endif
        }
    }
};

//=============================================================================
} // namespace cudax
//=============================================================================
