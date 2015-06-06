#version 330
in vec3 fnormal;
out vec4 out_color;
const vec3 vdir = vec3(0,0,1);

uniform vec3 color;

void main(){
    float shaded = abs(dot(fnormal, vdir));
    out_color = vec4(shaded*color,1); ///< Colored shaded    
    // out_color = vec4(shaded,0,0,1); /// RED shaded 
    // out_color = vec4(1,0,0,1); ///solid
    // out_color = vec4(color, 1);
}
