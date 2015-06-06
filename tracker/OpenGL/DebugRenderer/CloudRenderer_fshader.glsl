#version 330
in vec3 fcolor;
out vec4 color;
const vec3 camera_dir = vec3(0,0,-1);
void main(){
    color = vec4(fcolor,1);
}
