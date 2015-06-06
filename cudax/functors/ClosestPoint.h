#include "cudax/kernel.h"
#include "cudax/cuda_glm.h"

//== NAMESPACE ================================================================
namespace cudax {
//=============================================================================

struct ClosestPoint {

    float *_data_cylinders; ///< raw
    bool do_backface_check;

public:

    ClosestPoint(){
        _data_cylinders = thrust::raw_pointer_cast(cylinder_segments->data());
        do_backface_check = settings->fit3D_backface_check;
    }

    //-------------------------------------------------------------//

    __device__
    float SAFE_DOT(glm::vec3 u, glm::vec3 v){
        return u.x*v.x + u.y*v.y + u.z*v.z;
    }

    __device__
    float SAFE_LENGTH(glm::vec3 u){
        assert(u.x==u.x);
        return sqrt(u.x*u.x + u.y*u.y + u.z*u.z);
    }

    __device__
    glm::vec3 SAFE_NORMALIZE(glm::vec3 v, glm::vec3 fallback){
        glm::vec3 ret = fallback;
        if(SAFE_DOT(v, v) > FLT_MIN)
            ret = glm::normalize(v);
        return ret;
    }

    //-------------------------------------------------------------//

    __device__
    void line_segment_distance(glm::vec3 point,
            const glm::vec3 &start, const glm::vec3 &end,
            glm::vec3 &p, float &d)
    {
        glm::vec3 d1 = point - start;
        glm::vec3 d2 = end - start;
        glm::vec3 min_v = start;

        float t = SAFE_DOT(d2, d2);

        if(t > FLT_MIN){
            t = SAFE_DOT(d1, d2) / t;
            if      (t > 1.0f) d1 = point - (min_v = end);
            else if (t > 0.0f) d1 = point - (min_v = start + d2*t);
        }

        p = min_v;
        assert(d1.x==d1.x);
        d = SAFE_LENGTH(d1);
    }

    //-------------------------------------------------------------//

    __device__
    void backface_check(glm::vec3 point, glm::vec3 &p,
            const glm::vec3 &n, const glm::vec3 &v,
            const glm::vec3 &start, const glm::vec3 &end,
            float r1, float r2, float r1y, float r2y, float l)
    {
        // check angle between normal and view direction
        float nv = SAFE_DOT(n, v);
        if(nv > 0) return; // point is front-facing

        // compute intersection planes
        // normal vectors n*, support vectors s*
        glm::vec3 n1, n2, n3, s1, s2, s3, c, h;
        {
            // start sphere plane
            n1 = n;
            s1 = start;

            // end sphere plane
            n2 = n;
            s2 = end;

            // cone segment plane
            c  = SAFE_NORMALIZE(end - start, v);
            h  = SAFE_NORMALIZE(glm::cross(v, c), v);
            n3 = SAFE_NORMALIZE(glm::cross(h, c), n1);
            s3 = start;
        }

        // project point to intersection planes
        glm::vec3 p1, p2, p3, o1, o2, o3;
        {
            p1 = point - SAFE_DOT(point - s1, n1) * n1;
            p2 = point - SAFE_DOT(point - s2, n2) * n2;
            p3 = point - SAFE_DOT(point - s3, n3) * n3;

            glm::vec3 x(1,0,0), y(0,1,0);

            if(fabs(SAFE_DOT(y, n1)) != 1.0f){
                o1 = SAFE_NORMALIZE(y - SAFE_DOT(y, n1) * n1, y);
            }else{
                o1 = SAFE_NORMALIZE(x - SAFE_DOT(x, n1) * n1, y);
            }

            if(fabs(SAFE_DOT(y, n2)) != 1.0f){
                o2 = SAFE_NORMALIZE(y - SAFE_DOT(y, n2) * n2, y);
            }else{
                o2 = SAFE_NORMALIZE(x - SAFE_DOT(x, n2) * n2, y);
            }

            if(fabs(SAFE_DOT(y, n3)) != 1.0f){
                o3 = SAFE_NORMALIZE(y - SAFE_DOT(y, n3) * n3, y);
            }else{
                o3 = SAFE_NORMALIZE(x - SAFE_DOT(x, n3) * n3, y);
            }
        }

        // project onto intersection curves
        glm::vec3 u1, v1, u2, v2, u3, v3;
        {
            // start sphere
            u1 = p1 - s1;
            v1 = SAFE_NORMALIZE(u1, o1) * r1;
            p1 -= u1 - v1;

            // end sphere
            u2 = p2 - s2;
            v2 = SAFE_NORMALIZE(u2, o2) * r2;
            p2 -= u2 - v2;

            // cone segment
            glm::vec3 s_; float d;
            line_segment_distance(p3, start, end, s_, d);
            glm::vec3 tmp = end-start; assert(tmp.x==tmp.x);
            float le = SAFE_LENGTH(end - start);
            glm::vec3 tmp2 = s_-start; assert(tmp2.x==tmp2.x);
            float lp = SAFE_LENGTH(s_ - start);
            float t  = lp / le;
            float r_ = t * r2  + (1 - t) * r1;
            float ry = t * r2y + (1 - t) * r1y;
            float ratio = ry/r_;
            u3 = p3 - s_;
            v3 = SAFE_NORMALIZE(u3, o3) * r_ * ratio;
            p3 -= u3 - v3;
        }

        // select closest point to query
        {
            glm::vec3 tmp3 = p1-point; assert(tmp3.x==tmp3.x);
            float d1 = SAFE_LENGTH(p1 - point);
            glm::vec3 tmp4 = p2-point; assert(tmp4.x==tmp4.x);
            float d2 = SAFE_LENGTH(p2 - point);
            glm::vec3 tmp5 = p3-point; assert(tmp5.x==tmp5.x);
            float d3 = SAFE_LENGTH(p3 - point);

            p = (d1 < d2) ? ((d1 < d3) ? p1 : p3) : (d2 < d3) ? p2 : p3;
        }
    }

    //-------------------------------------------------------------//

    __device__
    void cylinder_distance(glm::vec3 point,
            const glm::vec3 &start, const glm::vec3 &end,
            float r1, float r2, float r1y, float r2y, float l,
            const glm::mat4 &T, const glm::mat4 &Ti,
            glm::vec3 &p, glm::vec3 &n, float &d)
    {
        // compute projection on center line segment
        line_segment_distance(point, start, end, p, d);

        // interpolate start/end radius
        glm::vec3 tmp1 = end-start; assert(tmp1.x==tmp1.x);
        float le = SAFE_LENGTH(end - start);
        glm::vec3 tmp2 = p-start; assert(tmp2.x==tmp2.x);
        float lp = SAFE_LENGTH(p - start);
        float t  = lp / le;
        float r  = t * r2  + (1 - t) * r1;
        float ry = t * r2y + (1 - t) * r1y;
        float ratio = ry/r;

        // scale from elliptical to circular
        glm::vec3 p0 = point;
        glm::vec3 pt = glm::vec3(Ti * glm::vec4(point, 1));
        pt.x *= ratio;
        point = glm::vec3(T * glm::vec4(pt, 1));

        // project onto cylindroid surface
        glm::vec3 view(0,0,-1);
        n = SAFE_NORMALIZE(point - p, -view);
        p += n * r * ratio;

#define ENABLE_BACKFACE_CHECK
#ifdef ENABLE_BACKFACE_CHECK
        // only allow front-facing points
        if(do_backface_check)
            backface_check(point, p, n, view, start, end, r1, r2, r1y, r2y, l);
#endif

        // unscale from circular to elliptical
        pt = glm::vec3(Ti * glm::vec4(p, 1));
        pt.x /= ratio;
        p = glm::vec3(T * glm::vec4(pt, 1));

        // compute correct normal at this point
        pt = glm::vec3(Ti * glm::vec4(n, 0));
        pt.x *= ratio;
        n = SAFE_NORMALIZE(glm::vec3(T * glm::vec4(pt, 0)), -view);

        // compute distance
        glm::vec3 tmp3 = p-p0; assert(tmp3.x==tmp3.x);
        if(ratio!=1.0f) d = SAFE_LENGTH(p - p0);
    }

    //-------------------------------------------------------------//

    __device__
    void closest_point(const float4 &p_sensor, float4 &p_model, float4 &n_model, int &joint_id){
        p_model = p_sensor;
        joint_id = -1;
        float min_dist = FLT_MAX;

        for(int i = 0; i < SEGMENT_JOINTS; ++i){
            int id, k = 0;
            glm::vec3 start, end;
            float r1, r2, r1y, r2y, l;
            glm::mat4 T, Ti;

            // joint id
            id = (int)_data_cylinders[i*SEGMENT_VALUES + k++];
            if(id == -1) continue;

            // start position
            start.x = _data_cylinders[i*SEGMENT_VALUES + k++];
            start.y = _data_cylinders[i*SEGMENT_VALUES + k++];
            start.z = _data_cylinders[i*SEGMENT_VALUES + k++];

            // end position
            end.x = _data_cylinders[i*SEGMENT_VALUES + k++];
            end.y = _data_cylinders[i*SEGMENT_VALUES + k++];
            end.z = _data_cylinders[i*SEGMENT_VALUES + k++];

            // segment parameters
            r1  = _data_cylinders[i*SEGMENT_VALUES + k++];
            r2  = _data_cylinders[i*SEGMENT_VALUES + k++];
            r1y = _data_cylinders[i*SEGMENT_VALUES + k++];
            r2y = _data_cylinders[i*SEGMENT_VALUES + k++];
            l   = _data_cylinders[i*SEGMENT_VALUES + k++];

            // joint transform matrices
            for(int j = 0; j < 16; ++j){
                int r = j % 4;
                int c = j / 4;
                T[c][r] = _data_cylinders[i*SEGMENT_VALUES + k];
                Ti[c][r] = _data_cylinders[i*SEGMENT_VALUES + k + 16];
                ++k;
            }

            float d;
            glm::vec3 p, n;
            glm::vec3 point(p_sensor.x, p_sensor.y, p_sensor.z);
            cylinder_distance(point, start, end, r1, r2, r1y, r2y, l, T, Ti, p, n, d);

            if(d < min_dist){
                min_dist = d;
                joint_id = id;

                p_model.x = p.x;
                p_model.y = p.y;
                p_model.z = p.z;
                p_model.w = 0;

                n_model.x = n.x;
                n_model.y = n.y;
                n_model.z = n.z;
                n_model.w = 0;
            }
        }
    }

    //-------------------------------------------------------------//

    __device__
    void relocate_target(float4 &p_sensor, float4 &p_model, float4 &n_model){
        glm::vec3 ps(p_sensor.x, p_sensor.y, p_sensor.z);
        glm::vec3 pm(p_model.x, p_model.y, p_model.z);
        glm::vec3 nm(n_model.x, n_model.y, n_model.z);
        glm::vec3 p = pm + SAFE_DOT(ps - pm, nm) * nm;
        p_sensor.x = p.x;
        p_sensor.y = p.y;
        p_sensor.z = p.z;
    }

    //-------------------------------------------------------------//

    __device__
    bool is_nan(float4 n){
        return n.x != n.x
            || n.y != n.y
            || n.z != n.z;
    }

    //-------------------------------------------------------------//

    __device__
    bool is_valid(float4 &p_sensor, float4 &p_model, float4 &n_model, int joint_id){
        return joint_id != -1 && !is_nan(p_sensor) && !is_nan(p_model) && !is_nan(n_model);
    }

    //-------------------------------------------------------------//

};

//=============================================================================
} // namespace cudax
//=============================================================================
