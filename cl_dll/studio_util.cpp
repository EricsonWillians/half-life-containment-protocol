//========= Copyright (c) 1996-2002, Valve LLC, All rights reserved. ============
//
// Purpose: Studio utility functions with enhanced robustness and error handling
//
// $NoKeywords: $
//=============================================================================

#include "hud.h"
#include "cl_util.h"
#include "const.h"
#include "com_model.h"
#include "studio_util.h"
#include <cmath>
#include <cfloat>
#include <cassert>
#include <cstring>

// Robust mathematical constants
#ifndef M_PI_F
#define M_PI_F 3.14159265358979323846f
#endif

// Safe epsilon values for floating point comparisons
#define STUDIO_EPSILON 1e-6f
#define STUDIO_QUATERNION_EPSILON 1e-9f
#define STUDIO_ANGLE_EPSILON 1e-8f

// Safe angle limits (in radians)
#define STUDIO_MAX_ANGLE (2.0f * M_PI_F)
#define STUDIO_HALF_PI (M_PI_F * 0.5f)

// Utility macros for robust floating point operations
#define STUDIO_IS_FINITE(x) (std::isfinite(x))
#define STUDIO_IS_NORMAL(x) (std::isnormal(x) || (x) == 0.0f)
#define STUDIO_SAFE_NORMALIZE_EPSILON 1e-12f

// Safe utility functions
namespace StudioUtil {
    inline bool IsValidFloat(float f) {
        return STUDIO_IS_FINITE(f) && !std::isnan(f);
    }
    
    inline bool IsValidVector(const float* v, int size) {
        if (!v) return false;
        for (int i = 0; i < size; ++i) {
            if (!IsValidFloat(v[i])) return false;
        }
        return true;
    }
    
    inline float SafeNormalize(float value, float min_val = -FLT_MAX, float max_val = FLT_MAX) {
        if (!IsValidFloat(value)) return 0.0f;
        return (value < min_val) ? min_val : (value > max_val) ? max_val : value;
    }
    
    inline float SafeAcos(float value) {
        value = SafeNormalize(value, -1.0f, 1.0f);
        return std::acos(value);
    }
    
    inline float SafeSin(float angle) {
        if (!IsValidFloat(angle)) return 0.0f;
        // Normalize angle to prevent precision issues
        while (angle > STUDIO_MAX_ANGLE) angle -= STUDIO_MAX_ANGLE;
        while (angle < -STUDIO_MAX_ANGLE) angle += STUDIO_MAX_ANGLE;
        return std::sin(angle);
    }
    
    inline float SafeCos(float angle) {
        if (!IsValidFloat(angle)) return 1.0f;
        // Normalize angle to prevent precision issues
        while (angle > STUDIO_MAX_ANGLE) angle -= STUDIO_MAX_ANGLE;
        while (angle < -STUDIO_MAX_ANGLE) angle += STUDIO_MAX_ANGLE;
        return std::cos(angle);
    }
}

/*
====================
AngleMatrix
Enhanced with input validation, overflow protection, and precision safeguards
====================
*/
void AngleMatrix(const float *angles, float (*matrix)[4])
{
    // Input validation
    if (!angles || !matrix) {
        // If matrix is valid, initialize to identity
        if (matrix) {
            memset(matrix, 0, sizeof(float) * 3 * 4);
            matrix[0][0] = matrix[1][1] = matrix[2][2] = 1.0f;
        }
        return;
    }
    
    // Validate input angles
    if (!StudioUtil::IsValidVector(angles, 3)) {
        // Initialize to identity matrix on invalid input
        memset(matrix, 0, sizeof(float) * 3 * 4);
        matrix[0][0] = matrix[1][1] = matrix[2][2] = 1.0f;
        return;
    }
    
    // Convert angles to radians with overflow protection
    const float deg_to_rad = M_PI_F / 180.0f;
    float yaw_rad = StudioUtil::SafeNormalize(angles[YAW] * deg_to_rad);
    float pitch_rad = StudioUtil::SafeNormalize(angles[PITCH] * deg_to_rad);
    float roll_rad = StudioUtil::SafeNormalize(angles[ROLL] * deg_to_rad);
    
    // Calculate trigonometric values safely
    float sy = StudioUtil::SafeSin(yaw_rad);
    float cy = StudioUtil::SafeCos(yaw_rad);
    float sp = StudioUtil::SafeSin(pitch_rad);
    float cp = StudioUtil::SafeCos(pitch_rad);
    float sr = StudioUtil::SafeSin(roll_rad);
    float cr = StudioUtil::SafeCos(roll_rad);
    
    // Build rotation matrix (YAW * PITCH * ROLL)
    matrix[0][0] = cp * cy;
    matrix[1][0] = cp * sy;
    matrix[2][0] = -sp;
    
    matrix[0][1] = sr * sp * cy + cr * (-sy);
    matrix[1][1] = sr * sp * sy + cr * cy;
    matrix[2][1] = sr * cp;
    
    matrix[0][2] = cr * sp * cy + (-sr) * (-sy);
    matrix[1][2] = cr * sp * sy + (-sr) * cy;
    matrix[2][2] = cr * cp;
    
    // Clear translation components
    matrix[0][3] = 0.0f;
    matrix[1][3] = 0.0f;
    matrix[2][3] = 0.0f;
    
    // Validate output matrix
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 4; ++j) {
            if (!StudioUtil::IsValidFloat(matrix[i][j])) {
                matrix[i][j] = (i == j && j < 3) ? 1.0f : 0.0f;
            }
        }
    }
}

/*
====================
VectorCompare
Enhanced with epsilon-based comparison and null pointer protection
====================
*/
int VectorCompare(const float *v1, const float *v2)
{
    // Null pointer protection
    if (!v1 || !v2) {
        return (v1 == v2) ? 1 : 0;
    }
    
    // Validate input vectors
    if (!StudioUtil::IsValidVector(v1, 3) || !StudioUtil::IsValidVector(v2, 3)) {
        return 0;
    }
    
    // Use epsilon-based comparison for floating point values
    for (int i = 0; i < 3; ++i) {
        if (std::abs(v1[i] - v2[i]) > STUDIO_EPSILON) {
            return 0;
        }
    }
    
    return 1;
}

/*
====================
CrossProduct
Enhanced with input validation and overflow protection
====================
*/
void CrossProduct(const float *v1, const float *v2, float *cross)
{
    // Input validation
    if (!v1 || !v2 || !cross) {
        if (cross) {
            cross[0] = cross[1] = cross[2] = 0.0f;
        }
        return;
    }
    
    // Validate input vectors
    if (!StudioUtil::IsValidVector(v1, 3) || !StudioUtil::IsValidVector(v2, 3)) {
        cross[0] = cross[1] = cross[2] = 0.0f;
        return;
    }
    
    // Calculate cross product with temporary storage to handle aliasing
    float temp[3];
    temp[0] = v1[1] * v2[2] - v1[2] * v2[1];
    temp[1] = v1[2] * v2[0] - v1[0] * v2[2];
    temp[2] = v1[0] * v2[1] - v1[1] * v2[0];
    
    // Validate results and copy
    for (int i = 0; i < 3; ++i) {
        cross[i] = StudioUtil::IsValidFloat(temp[i]) ? temp[i] : 0.0f;
    }
}

/*
====================
VectorTransform
Enhanced with comprehensive input validation
====================
*/
void VectorTransform(const float *in1, float in2[3][4], float *out)
{
    // Input validation
    if (!in1 || !in2 || !out) {
        if (out) {
            out[0] = out[1] = out[2] = 0.0f;
        }
        return;
    }
    
    // Validate input vector
    if (!StudioUtil::IsValidVector(in1, 3)) {
        out[0] = out[1] = out[2] = 0.0f;
        return;
    }
    
    // Validate transformation matrix
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 4; ++j) {
            if (!StudioUtil::IsValidFloat(in2[i][j])) {
                out[0] = out[1] = out[2] = 0.0f;
                return;
            }
        }
    }
    
    // Perform transformation with temporary storage
    float temp[3];
    for (int i = 0; i < 3; ++i) {
        temp[i] = in1[0] * in2[i][0] + in1[1] * in2[i][1] + in1[2] * in2[i][2] + in2[i][3];
        if (!StudioUtil::IsValidFloat(temp[i])) {
            temp[i] = 0.0f;
        }
    }
    
    // Copy results
    out[0] = temp[0];
    out[1] = temp[1];
    out[2] = temp[2];
}

/*
====================
ConcatTransforms
Enhanced with full matrix validation and overflow protection
====================
*/
void ConcatTransforms(float in1[3][4], float in2[3][4], float out[3][4])
{
    // Input validation
    if (!in1 || !in2 || !out) {
        if (out) {
            memset(out, 0, sizeof(float) * 3 * 4);
            out[0][0] = out[1][1] = out[2][2] = 1.0f;
        }
        return;
    }
    
    // Validate input matrices
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 4; ++j) {
            if (!StudioUtil::IsValidFloat(in1[i][j]) || !StudioUtil::IsValidFloat(in2[i][j])) {
                memset(out, 0, sizeof(float) * 3 * 4);
                out[0][0] = out[1][1] = out[2][2] = 1.0f;
                return;
            }
        }
    }
    
    // Use temporary matrix to handle potential aliasing
    float temp[3][4];
    
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 4; ++j) {
            if (j < 3) {
                // Rotation part
                temp[i][j] = in1[i][0] * in2[0][j] + 
                           in1[i][1] * in2[1][j] + 
                           in1[i][2] * in2[2][j];
            } else {
                // Translation part
                temp[i][j] = in1[i][0] * in2[0][j] + 
                           in1[i][1] * in2[1][j] + 
                           in1[i][2] * in2[2][j] + in1[i][j];
            }
            
            // Validate result
            if (!StudioUtil::IsValidFloat(temp[i][j])) {
                temp[i][j] = (i == j && j < 3) ? 1.0f : 0.0f;
            }
        }
    }
    
    // Copy result
    memcpy(out, temp, sizeof(float) * 3 * 4);
}

/*
====================
AngleQuaternion
Enhanced with input validation and normalized output
====================
*/
void AngleQuaternion(float *angles, vec4_t quaternion)
{
    // Input validation
    if (!angles || !quaternion) {
        if (quaternion) {
            quaternion[0] = quaternion[1] = quaternion[2] = 0.0f;
            quaternion[3] = 1.0f; // Identity quaternion
        }
        return;
    }
    
    // Validate input angles
    if (!StudioUtil::IsValidVector(angles, 3)) {
        quaternion[0] = quaternion[1] = quaternion[2] = 0.0f;
        quaternion[3] = 1.0f;
        return;
    }
    
    // Convert to half angles with normalization
    float half_angles[3];
    for (int i = 0; i < 3; ++i) {
        half_angles[i] = StudioUtil::SafeNormalize(angles[i] * 0.5f);
    }
    
    // Calculate trigonometric values
    float sr = StudioUtil::SafeSin(half_angles[0]);
    float cr = StudioUtil::SafeCos(half_angles[0]);
    float sp = StudioUtil::SafeSin(half_angles[1]);
    float cp = StudioUtil::SafeCos(half_angles[1]);
    float sy = StudioUtil::SafeSin(half_angles[2]);
    float cy = StudioUtil::SafeCos(half_angles[2]);
    
    // Calculate quaternion components
    quaternion[0] = sr * cp * cy - cr * sp * sy; // X
    quaternion[1] = cr * sp * cy + sr * cp * sy; // Y
    quaternion[2] = cr * cp * sy - sr * sp * cy; // Z
    quaternion[3] = cr * cp * cy + sr * sp * sy; // W
    
    // Validate and normalize quaternion
    float length_sq = 0.0f;
    for (int i = 0; i < 4; ++i) {
        if (!StudioUtil::IsValidFloat(quaternion[i])) {
            quaternion[0] = quaternion[1] = quaternion[2] = 0.0f;
            quaternion[3] = 1.0f;
            return;
        }
        length_sq += quaternion[i] * quaternion[i];
    }
    
    // Normalize if needed
    if (length_sq > STUDIO_SAFE_NORMALIZE_EPSILON) {
        float inv_length = 1.0f / std::sqrt(length_sq);
        for (int i = 0; i < 4; ++i) {
            quaternion[i] *= inv_length;
        }
    } else {
        quaternion[0] = quaternion[1] = quaternion[2] = 0.0f;
        quaternion[3] = 1.0f;
    }
}

/*
====================
QuaternionSlerp
Enhanced with comprehensive input validation and numerical stability
====================
*/
void QuaternionSlerp(vec4_t p, vec4_t q, float t, vec4_t qt)
{
    // Input validation
    if (!p || !q || !qt) {
        if (qt) {
            qt[0] = qt[1] = qt[2] = 0.0f;
            qt[3] = 1.0f;
        }
        return;
    }
    
    // Validate quaternions
    if (!StudioUtil::IsValidVector(p, 4) || !StudioUtil::IsValidVector(q, 4)) {
        qt[0] = qt[1] = qt[2] = 0.0f;
        qt[3] = 1.0f;
        return;
    }
    
    // Clamp interpolation parameter
    t = StudioUtil::SafeNormalize(t, 0.0f, 1.0f);
    
    // Copy quaternions to avoid modifying inputs
    vec4_t q_copy;
    memcpy(q_copy, q, sizeof(vec4_t));
    
    // Choose shortest path
    float dot = p[0] * q_copy[0] + p[1] * q_copy[1] + p[2] * q_copy[2] + p[3] * q_copy[3];
    
    if (dot < 0.0f) {
        for (int i = 0; i < 4; ++i) {
            q_copy[i] = -q_copy[i];
        }
        dot = -dot;
    }
    
    // Clamp dot product to valid range
    dot = StudioUtil::SafeNormalize(dot, -1.0f, 1.0f);
    
    float sclp, sclq;
    
    // Choose interpolation method based on angle
    if (dot > 0.9995f) {
        // Linear interpolation for very close quaternions
        sclp = 1.0f - t;
        sclq = t;
    } else {
        // Spherical linear interpolation
        float omega = StudioUtil::SafeAcos(std::abs(dot));
        float sinom = StudioUtil::SafeSin(omega);
        
        if (sinom > STUDIO_QUATERNION_EPSILON) {
            sclp = StudioUtil::SafeSin((1.0f - t) * omega) / sinom;
            sclq = StudioUtil::SafeSin(t * omega) / sinom;
        } else {
            sclp = 1.0f - t;
            sclq = t;
        }
    }
    
    // Interpolate and validate result
    for (int i = 0; i < 4; ++i) {
        qt[i] = sclp * p[i] + sclq * q_copy[i];
        if (!StudioUtil::IsValidFloat(qt[i])) {
            qt[i] = (i == 3) ? 1.0f : 0.0f;
        }
    }
    
    // Normalize result
    float length_sq = qt[0] * qt[0] + qt[1] * qt[1] + qt[2] * qt[2] + qt[3] * qt[3];
    if (length_sq > STUDIO_SAFE_NORMALIZE_EPSILON) {
        float inv_length = 1.0f / std::sqrt(length_sq);
        for (int i = 0; i < 4; ++i) {
            qt[i] *= inv_length;
        }
    } else {
        qt[0] = qt[1] = qt[2] = 0.0f;
        qt[3] = 1.0f;
    }
}

/*
====================
QuaternionMatrix
Enhanced with input validation and orthogonality preservation
====================
*/
void QuaternionMatrix(vec4_t quaternion, float (*matrix)[4])
{
    // Input validation
    if (!quaternion || !matrix) {
        if (matrix) {
            memset(matrix, 0, sizeof(float) * 3 * 4);
            matrix[0][0] = matrix[1][1] = matrix[2][2] = 1.0f;
        }
        return;
    }
    
    // Validate quaternion
    if (!StudioUtil::IsValidVector(quaternion, 4)) {
        memset(matrix, 0, sizeof(float) * 3 * 4);
        matrix[0][0] = matrix[1][1] = matrix[2][2] = 1.0f;
        return;
    }
    
    // Normalize quaternion
    float length_sq = quaternion[0] * quaternion[0] + quaternion[1] * quaternion[1] + 
                     quaternion[2] * quaternion[2] + quaternion[3] * quaternion[3];
    
    vec4_t q;
    if (length_sq > STUDIO_SAFE_NORMALIZE_EPSILON) {
        float inv_length = 1.0f / std::sqrt(length_sq);
        for (int i = 0; i < 4; ++i) {
            q[i] = quaternion[i] * inv_length;
        }
    } else {
        q[0] = q[1] = q[2] = 0.0f;
        q[3] = 1.0f;
    }
    
    // Calculate matrix elements
    float xx = q[0] * q[0];
    float yy = q[1] * q[1];
    float zz = q[2] * q[2];
    float xy = q[0] * q[1];
    float xz = q[0] * q[2];
    float yz = q[1] * q[2];
    float wx = q[3] * q[0];
    float wy = q[3] * q[1];
    float wz = q[3] * q[2];
    
    // Build rotation matrix
    matrix[0][0] = 1.0f - 2.0f * (yy + zz);
    matrix[0][1] = 2.0f * (xy - wz);
    matrix[0][2] = 2.0f * (xz + wy);
    matrix[0][3] = 0.0f;
    
    matrix[1][0] = 2.0f * (xy + wz);
    matrix[1][1] = 1.0f - 2.0f * (xx + zz);
    matrix[1][2] = 2.0f * (yz - wx);
    matrix[1][3] = 0.0f;
    
    matrix[2][0] = 2.0f * (xz - wy);
    matrix[2][1] = 2.0f * (yz + wx);
    matrix[2][2] = 1.0f - 2.0f * (xx + yy);
    matrix[2][3] = 0.0f;
    
    // Validate all matrix elements
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 4; ++j) {
            if (!StudioUtil::IsValidFloat(matrix[i][j])) {
                matrix[i][j] = (i == j && j < 3) ? 1.0f : 0.0f;
            }
        }
    }
}

/*
====================
MatrixCopy
Enhanced with comprehensive input validation
====================
*/
void MatrixCopy(float in[3][4], float out[3][4])
{
    // Input validation
    if (!in || !out) {
        if (out) {
            memset(out, 0, sizeof(float) * 3 * 4);
            out[0][0] = out[1][1] = out[2][2] = 1.0f;
        }
        return;
    }
    
    // Validate input matrix and copy with validation
    bool valid = true;
    for (int i = 0; i < 3 && valid; ++i) {
        for (int j = 0; j < 4 && valid; ++j) {
            if (!StudioUtil::IsValidFloat(in[i][j])) {
                valid = false;
            }
        }
    }
    
    if (valid) {
        memcpy(out, in, sizeof(float) * 3 * 4);
    } else {
        // Initialize to identity matrix on invalid input
        memset(out, 0, sizeof(float) * 3 * 4);
        out[0][0] = out[1][1] = out[2][2] = 1.0f;
    }
}