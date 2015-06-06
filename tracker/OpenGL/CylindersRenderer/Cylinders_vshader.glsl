#version 330

uniform mat4 view; ///< not identity because of freeform viewer
uniform mat4 view_projection;
uniform mat4 joint_transform;

in vec3 vpoint;
in vec3 vnormal;
out vec3 fnormal;

void main() {  
    gl_Position = view_projection * joint_transform * vec4(vpoint, 1.0);
    ///< this is only possible because model_view is uniform scale!!
    fnormal = normalize( mat3(view)*mat3(joint_transform)*vec3(vnormal));
} 
