#version 330
uniform mat4 model_view_projection;
in vec3 vpoint;
out vec2 uv;

mat4 MVP = mat4(0);

void main() 
{  
    /// Map [0,1]^2 to [-1,1]^2 and flip matrix
    MVP[3][0] = -1;
    MVP[3][1] = +1;
    MVP[0][0] = +2.0;
    MVP[1][1] = -2.0;
    MVP[3][3] = 1;
    
    gl_Position = MVP*vec4(vpoint, 1.0);
    uv = vpoint.xy;
}
