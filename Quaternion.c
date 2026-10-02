/*
 * Quaternion.c
 *
 *  Created on: May 24, 2026
 *      Author: vultra_dev
 */

#include "Quaternion.h"
/**
 * @brief  return 1/x or 0 if x < EPS.
 *         Centralizes the division-by-near-zero protection.
 */
static float save_inv(float x){
    assert(x >= 0.0f);
    return (x > QUAT_EPS) ? (1.0f / x) : 0.0f;
}

void Quat_Normalize(Quaternion_t *q){
    assert(q != NULL);
 
    const float inv_n = save_inv(sqrtf(Quat_NormSq(*q)));
 
    /* save_inv() returns 0 for a near-zero norm: leave q unchanged
     * rather than zeroing it out, since a zero quaternion is not a
     * valid orientation and would corrupt ahrs->q on the next use. */
    if (inv_n > 0.0f) {
        q->w *= inv_n;
        q->x *= inv_n;
        q->y *= inv_n;
        q->z *= inv_n;
    }
}   

/**
 * Quat_Multiply
 */
void Quat_Multiply(const Quaternion_t *q1,const Quaternion_t *q2, Quaternion_t *res)
{
    /* assert non-null pointers */
    assert(q1  != NULL);
    assert(q2  != NULL);
    assert(res != NULL);

    /* Cache inputs so res may alias q1 or q2 safely */
    const float w1 = q1->w, x1 = q1->x, y1 = q1->y, z1 = q1->z;
    const float w2 = q2->w, x2 = q2->x, y2 = q2->y, z2 = q2->z;

    res->w = w1*w2 - x1*x2 - y1*y2 - z1*z2;
    res->x = w1*x2 + x1*w2 + y1*z2 - z1*y2;
    res->y = w1*y2 - x1*z2 + y1*w2 + z1*x2;
    res->z = w1*z2 + x1*y2 - y1*x2 + z1*w2;
}

/**
 * Quat_Reciprocal
 */
void Quat_Reciprocal(const Quaternion_t *q, Quaternion_t *res){
    assert(q   != NULL);
    assert(res != NULL);
    const float inv_norm_sq = save_inv(Quat_NormSq(*q));

    res->w =  q->w * inv_norm_sq;
    res->x = -q->x * inv_norm_sq;
    res->y = -q->y * inv_norm_sq;
    res->z = -q->z * inv_norm_sq;
}

/**
 * Quat_ToEuler
 */

void Quat_ToEuler(const Quaternion_t *q, Euler_t *angle){
    assert(q     != NULL);
    assert(angle != NULL);

    const float ww = q->w * q->w;
    const float xx = q->x * q->x;
    const float yy = q->y * q->y;
    const float zz = q->z * q->z;

    /*Roll  φ: rotation about X */
    angle->roll = atan2f(2.0f * (q->w * q->x + q->y * q->z), ww - xx - yy + zz);

     /* Pitch θ: rotation about Y — clamp for gimbal lock (±90°) */
    const float sinp = 2.0f * (q->w * q->y - q->z * q->x);


    if      (sinp >=  1.0f) { angle->pitch  =  HALF_PI;     }
    else if (sinp <= -1.0f) { angle->pitch  = -HALF_PI;     }
    else                    { angle->pitch  =  asinf(sinp); }

    /* Yaw   ψ: rotation about Z  ← q->w * q->z */
    angle->yaw = atan2f(2.0f * (q->w * q->z + q->x * q->y), ww + xx -yy - zz);

}

/**
 * Quat_RotateVector
 */
void Quat_RotateVector(const Quaternion_t *q, const float v_in[3], float v_out[3]){

    assert(q     != NULL);
    assert(v_in  != NULL);
    assert(v_out != NULL);

    const float tx = 2.0f * (q->y * v_in[2] - q->z * v_in[1]);
    const float ty = 2.0f * (q->z * v_in[0] - q->x * v_in[2]);
    const float tz = 2.0f * (q->x * v_in[1] - q->y * v_in[0]);

    v_out[0] = v_in[0] + q->w * tx + (q->y * tz - q->z * ty);
    v_out[1] = v_in[1] + q->w * ty + (q->z * tx - q->x * tz);
    v_out[2] = v_in[2] + q->w * tz + (q->x * ty - q->y * tx);

}


