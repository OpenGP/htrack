#version 330
uniform sampler2D tex;
in vec2 uv;
out vec4 color;

void main() {
    // if(uv.x<.001 || uv.x>.99 || uv.y<.01 || uv.y>.99 ){ color = vec4(0,0,0,1); return; }
    color = vec4( texture(tex, uv).rgb, 1.0 );
}

