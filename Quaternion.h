/*
 * Quaternion.h
 *
 *  Created on: May 24, 2026
 *      Author: vultra_dev
 */


#ifndef QUATERNION_H
#define QUATERNION_H


#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <math.h>
#include <assert.h>

#define QUAT_EPS = 1e-6f; /* near zero guard */
#define HALF_PI  = 1.5707964f; /* pi/2 for glimbal lock */

/**
 * Quaternion Library
 * Convention q = w + xi + yj + zk
 */

/**
 * @brief Data structure of a quaternion
 * @struct Quaternion_t
 */
typedef struct {
    float w; /* scalar(real) part */
    float x, y, z; /* vector(imaginary) part */
} Quaternion_t;

/**
 * @brief Euler angle to demonstrate the output angle
 * @struct Euler_t
 */

typedef struct {
    float roll;  /**Rotation around X-Axis[rad] */
    float pitch; /**Rotation around Y-Axis[rad] */
    float yaw;   /**Rotation around Z-Axis[rad] */
} Euler_t;


/**
 * @brief  Create a quaternion from four components.
 * @param  w  Scalar part.
 * @param  x  i-component.
 * @param  y  j-component.
 * @param  z  k-component.
 * @return Initialized Quaternion_t.
 */

static inline Quaternion_t Quat_Create(float w, float x, float y, float z){
    Quaternion_t q;
    q.w = w;
    q.x = x;
    q.y = y;
    q.z = z;
    return q;
}

/**
 * @brief  Return the identity quaternion  q = 1 + 0i + 0j + 0k.
 */
static inline Quaternion_t Quat_Identity(void)
{
    return Quat_Create(1.0f, 0.0f, 0.0f, 0.0f);
}


/**
 * @brief  Component-wise addition:  q_out = q1 + q2.
 *
 * Formula:
 *   (w1+w2) + (x1+x2)i + (y1+y2)j + (z1+z2)k
 */
static inline Quaternion_t Quat_Add(Quaternion_t q1, Quaternion_t q2){
    return Quat_Create(q1.w + q2.w, q1.x + q2.x, q1.y + q2.y, q1.z + q2.z);
}


/**
 * @brief  Scale all components by a scalar:  q_out = s * q.
 */
static inline Quaternion_t Quat_Scale(Quaternion_t q, float s)
{
    return Quat_Create(q.w * s, q.x * s, q.y * s, q.z * s);
}

/**
 * @brief  Squared Euclidean norm:  ||q||^2 = w^2 + x^2 + y^2 + z^2.
 */
static inline float Quat_NormSq(Quaternion_t q){
    return q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
}
/**
 * @brief Conjugation of a quaternion
 * Formula: q = w - x * i - y * j - z * k
 *
 */
static inline Quaternion_t Quat_Conjugate(Quaternion_t q){
    return Quat_Create(q.w, -q.x, -q.y, -q.z);
}


/**
 * @brief  Hamilton (non-commutative) product:  *res = (*q1) * (*q2).
 *
 * Formula:
 *   w = w1·w2 − x1·x2 − y1·y2 − z1·z2
 *   x = w1·x2 + x1·w2 + y1·z2 − z1·y2
 *   y = w1·y2 − x1·z2 + y1·w2 + z1·x2
 *   z = w1·z2 + x1·y2 − y1·x2 + z1·w2
 * @param  q1   Left operand  (non-NULL).
 * @param  q2   Right operand (non-NULL).
 * @param  res  Output; may alias q1 or q2.
 */

void Quat_Multiply(const Quaternion_t *q1, const Quaternion_t *q2, Quaternion_t *res);


/**
 * @brief  Normalize q in-place so that ||q|| == 1.
 *         No-op if ||q|| < QUAT_EPS to avoid division by zero.
 *
 * @param  q  Quaternion to normalize (non-NULL).
 */
void Quat_Normalize(Quaternion_t *q);


/**
 * @brief  Multiplicative inverse (reciprocal):  *res = q^{-1} = q* / ||q||^2.
 *         For a unit quaternion this equals the conjugate.
 *
 * @param  q    Input quaternion (non-NULL, ||q|| > QUAT_EPS).
 * @param  res  Output (non-NULL).
 */
void Quat_Reciprocal(const Quaternion_t *q, Quaternion_t *res);

/**
 * @brief  Convert a unit quaternion to ZYX Euler angles (roll-pitch-yaw).
 *
 * Convention (aerospace / ZYX intrinsic):
 *   roll  φ = atan2( 2(wx + yz),  w²−x²−y²+z² )
 *   pitch θ = arcsin( 2(wy − zx) )   [clamped for gimbal lock]
 *   yaw   ψ = atan2( 2(wz + xy),  w²+x²−y²−z² )
 *
 * @param  q      Unit quaternion (non-NULL).
 * @param  angle  Output Euler angles in radians (non-NULL).
 */
void Quat_ToEuler(const Quaternion_t *q, Euler_t *angle);

/**
 * @brief  Rotate a 3-D vector by a unit quaternion:
 *         v_out = q * [0, v_in] * q^{-1}
 *
 * Optimized Rodriguez form (no full Hamilton product):
 *   t     = 2 * (q_vec × v_in)
 *   v_out = v_in + w·t + (q_vec × t)
 *
 * @param  q      Unit quaternion (non-NULL).
 * @param  v_in   Input  vector [3] (non-NULL).
 * @param  v_out  Output vector [3] (non-NULL, may alias v_in).
 */

void Quat_RotateVector(const Quaternion_t *q, const float v_in[3], float v_out[3]);



#ifdef __cplusplus
}
#endif

#endif /* QUATERNION_H */
