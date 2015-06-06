#version 330
uniform float instance_id; ///< technically a int, but not allowed in this GLSL version
in vec3 vpoint_camera; ///< interpolated 3D location in camera space
// in vec3 fnormal;

layout(location=0) out uint out_color;
layout(location=1) out vec4 out_extra;
layout(location=2) out vec4 out_norms;

void main(){
    out_color = uint(instance_id); ///< draw the instance_id

    ///--- [3D point, NaN ] @todo results in CUDA crash :(
    // out_extra = vec4(vpoint_camera, 0.0/0.0);     

    ///--- [3D point, isSilhouette ] @todo check if it's advantageous to do here?
    // out_extra = vec4(vpoint_camera, (instance_id<255)); 
    
    ///--- [3D point, squared distance from camera (origin) ]
    // out_extra = vec4(vpoint_camera, dot(vpoint_camera,vpoint_camera)); 
  
    //----------------------------------------------------------------
    //--- Screen space normals
    vec3 vx = normalize( dFdx(vpoint_camera) );
    vec3 vy = normalize( dFdy(vpoint_camera) );
    vec3 normal = -cross(vx, vy);
    out_norms = vec4( normal, 0 );
    //--- Normals from model (needs to be computed)
    // out_norms = vec4(fnormal,0);
    //----------------------------------------------------------------
    
    ///--- [3D point, normal front facing angle ]
    out_extra = vec4(vpoint_camera, dot(normal,vec3(0,0,-1))); 
}
 
