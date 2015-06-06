#version 330 core
///--- Stuff to colormap the depth mesh
uniform sampler1D colormap;
uniform float zNear;
uniform float zFar;
uniform usampler2D tex_depth;
uniform float enable_colormap;

///--- Stuff to discard not front-facing
in vec3 f_position;
uniform float discard_cosalpha_th; ///< .4?
uniform sampler2D tex_color; ///< sensor color texture

in vec2 frag_uv;             ///< UVs to access tex_color
uniform float alpha;         ///< alpha blending value
in float depth;              ///< interpolated depth value
in float discardme;          ///< used to discard grazing angle triangles
out vec4 color;

// paste in htrack gui:
//  cloud.set_discard_cosalpha_th(0.0)

/// @see https://www.opengl.org/wiki/GL_EXT_texture_integer
void main(){
    if(discardme>0) discard;
    
    //--- Discard out of z-range
    if(depth<zNear || depth>zFar) discard;

    vec3 vx = dFdx(f_position);
    vec3 vy = dFdy(f_position);   
    vec3 vray = normalize( f_position );
    vec3 n = normalize( cross(vx,vy) );
    float cosalpha = abs( dot(n, vray) );
    if( cosalpha<discard_cosalpha_th ) discard;
            
    if(enable_colormap>0)
    {
        ///--- Apply a colormap
        float range = zFar-zNear;
        float w = (depth-zNear)/range;
        color = vec4( texture(colormap, w).rgb, alpha);
    }
    else 
    {
        ///--- Apply the color texture
        color = vec4( texture(tex_color, frag_uv).rgb, alpha);
    }    
}


// float uv_depth = float( texture(tex_depth, frag_uv).x );
// if( abs(dFdx(uv_depth))>ddiff_th || abs(dFdy(uv_depth))>ddiff_th ) discard;
