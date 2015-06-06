#version 330
uniform usampler2D tex;
uniform sampler1D colormap;
in vec2 uv;
out vec4 color;
uniform uint znear;
uniform uint zfar;

uint dmin = znear; // 300U
uint dmax = zfar;  //1000U

void main() {
    uint val = texture(tex, uv).x;
    float range = dmax-dmin;
    float w = (val-dmin)/range;
    color = vec4( texture(colormap, w).rgb, 1);
    if(val<dmin || val>dmax) 
        color = vec4(0,0,0,1);
}
