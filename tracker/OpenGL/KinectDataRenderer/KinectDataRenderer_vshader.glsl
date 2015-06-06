#version 330
/// texture with depth from sensor
uniform usampler2D tex_depth; 
uniform float zNear;
uniform float zFar;
///--- 
uniform mat4 view_projection; ///< for visualization 
uniform mat3 inv_proj_matrix; ///< depth-to-3D conversion
in vec2 vpoint;
in vec2 uv;


out float depth;
out vec2 frag_uv;
out float discardme;
out vec3 f_position;

void main() {   
    ///--- Tex coords for color fetch
    frag_uv = uv;
    
    ///--- Depth evaluation
    depth = float( texture(tex_depth, uv).x );
    discardme = float(depth<zNear || depth>zFar);
    vec3 p = inv_proj_matrix * vec3(vpoint[0]*depth, vpoint[1]*depth, depth);
    gl_Position = view_projection * vec4(p[0],-p[1],p[2], 1.0);
    f_position = p;
    
    ///--- Splat size
    // gl_PointSize = 2; ///< Screen
    gl_PointSize = 4; ///< RETINA
}
   
