#version 330

// uniform mat4 view; ///< identity!!!
uniform mat4 view_projection;
uniform mat4 joint_transform;

in vec3 vpoint;
out vec3 vpoint_camera;

in vec3 vnormal;
// out vec3 fnormal;

void main() {  
    gl_Position = view_projection * joint_transform * vec4(vpoint, 1.0);
    vpoint_camera = vec3( joint_transform * vec4(vpoint, 1.0) );

    ///--- NOTE: vnormals were not re-computed, let's do it in screen space.
    // fnormal = normalize( /*mat3(view)*/ mat3(joint_transform)*vec3(vnormal));
}
