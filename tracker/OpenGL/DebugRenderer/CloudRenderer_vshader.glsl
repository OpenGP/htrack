#version 330
uniform mat4 view;
uniform mat4 view_projection;
in vec3 vpoint;
in vec3 vcolor;
out vec3 fcolor;

void main() {  
    gl_Position = view_projection * vec4(vpoint, 1.0);
    gl_PointSize = 3;
    fcolor = vcolor;
}
