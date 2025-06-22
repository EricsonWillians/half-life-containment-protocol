/***
 *
 *	Copyright (c) 1996-2002, Valve LLC. All rights reserved.
 *
 *	This product contains software technology licensed from Id
 *	Software, Inc. ("Id Technology").  Id Technology (c) 1996 Id Software, Inc.
 *	All Rights Reserved.
 *
 *   Use, distribution, and modification of this source code and/or resulting
 *   object code is restricted to non-commercial enhancements to products from
 *   Valve LLC.  All other use, distribution, or modification is prohibited
 *   without written permission from Valve LLC.
 *
 ****/

#ifndef UTIL_VECTOR_H
#define UTIL_VECTOR_H

/* C89/C++ compatibility headers */
#ifdef __cplusplus
extern "C"
{
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef __cplusplus
#if defined(_MSC_VER) && _MSC_VER >= 1600 /* VS2010+ */
#include <cmath>
#elif defined(__GNUC__) && defined(__cplusplus)
#include <cmath>
#else
#include <math.h>
#endif
#else
#include <math.h>
#endif

#ifdef __cplusplus
}
#endif

/* Type definitions */
typedef unsigned int func_t;
typedef int string_t;
typedef float vec_t;

/* Math utility macros for C89 compatibility */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef M_PI_F
#define M_PI_F 3.14159265358979323846f
#endif

/* Epsilon for floating point comparisons */
#ifndef EPSILON
#define EPSILON 1e-6f
#endif

/* Forward declarations - C++ only needs class forward declarations */
#ifdef __cplusplus
class Vector2D;
class Vector;
/* Note: No forward declaration for Vector3 - it will be a typedef */
#else
typedef struct Vector2D Vector2D;
typedef struct Vector Vector;
typedef struct Vector Vector3;
#endif

/*=========================================================
 * C-style vector operations - Header-only implementation
 *=========================================================*/

/* 2D Vector C functions - inline/static for header-only */
#ifdef __cplusplus
inline
#else
static
#endif
	void
	vec2d_set(float *v, float x, float y)
{
	if (v)
	{
		v[0] = x;
		v[1] = y;
	}
}

#ifdef __cplusplus
inline
#else
static
#endif
	void
	vec2d_add(const float *a, const float *b, float *result)
{
	if (a && b && result)
	{
		result[0] = a[0] + b[0];
		result[1] = a[1] + b[1];
	}
}

#ifdef __cplusplus
inline
#else
static
#endif
	void
	vec2d_sub(const float *a, const float *b, float *result)
{
	if (a && b && result)
	{
		result[0] = a[0] - b[0];
		result[1] = a[1] - b[1];
	}
}

#ifdef __cplusplus
inline
#else
static
#endif
	void
	vec2d_scale(const float *v, float scale, float *result)
{
	if (v && result)
	{
		result[0] = v[0] * scale;
		result[1] = v[1] * scale;
	}
}

#ifdef __cplusplus
inline
#else
static
#endif
	float
	vec2d_length(const float *v)
{
	return v ? (float)sqrt((double)(v[0] * v[0] + v[1] * v[1])) : 0.0f;
}

#ifdef __cplusplus
inline
#else
static
#endif
	float
	vec2d_dot(const float *a, const float *b)
{
	return (a && b) ? (a[0] * b[0] + a[1] * b[1]) : 0.0f;
}

#ifdef __cplusplus
inline
#else
static
#endif
	void
	vec2d_normalize(const float *v, float *result)
{
	if (v && result)
	{
		float len = vec2d_length(v);
		if (len < EPSILON)
		{
			result[0] = result[1] = 0.0f;
		}
		else
		{
			len = 1.0f / len;
			result[0] = v[0] * len;
			result[1] = v[1] * len;
		}
	}
}

#ifdef __cplusplus
inline
#else
static
#endif
	void
	vec2d_copy(const float *src, float *dst)
{
	if (src && dst)
	{
		dst[0] = src[0];
		dst[1] = src[1];
	}
}

#ifdef __cplusplus
inline
#else
static
#endif
	int
	vec2d_compare(const float *a, const float *b, float epsilon)
{
	if (!a || !b)
		return 0;
	return (fabs(a[0] - b[0]) < epsilon && fabs(a[1] - b[1]) < epsilon) ? 1 : 0;
}

/* 3D Vector C functions - inline/static for header-only */
#ifdef __cplusplus
inline
#else
static
#endif
	void
	vec3_set(float *v, float x, float y, float z)
{
	if (v)
	{
		v[0] = x;
		v[1] = y;
		v[2] = z;
	}
}

#ifdef __cplusplus
inline
#else
static
#endif
	void
	vec3_add(const float *a, const float *b, float *result)
{
	if (a && b && result)
	{
		result[0] = a[0] + b[0];
		result[1] = a[1] + b[1];
		result[2] = a[2] + b[2];
	}
}

#ifdef __cplusplus
inline
#else
static
#endif
	void
	vec3_sub(const float *a, const float *b, float *result)
{
	if (a && b && result)
	{
		result[0] = a[0] - b[0];
		result[1] = a[1] - b[1];
		result[2] = a[2] - b[2];
	}
}

#ifdef __cplusplus
inline
#else
static
#endif
	void
	vec3_scale(const float *v, float scale, float *result)
{
	if (v && result)
	{
		result[0] = v[0] * scale;
		result[1] = v[1] * scale;
		result[2] = v[2] * scale;
	}
}

#ifdef __cplusplus
inline
#else
static
#endif
	void
	vec3_cross(const float *a, const float *b, float *result)
{
	if (a && b && result)
	{
		/* Use temporary variables to handle in-place operations */
		float temp[3];
		temp[0] = a[1] * b[2] - a[2] * b[1];
		temp[1] = a[2] * b[0] - a[0] * b[2];
		temp[2] = a[0] * b[1] - a[1] * b[0];
		result[0] = temp[0];
		result[1] = temp[1];
		result[2] = temp[2];
	}
}

#ifdef __cplusplus
inline
#else
static
#endif
	float
	vec3_length(const float *v)
{
	return v ? (float)sqrt((double)(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])) : 0.0f;
}

#ifdef __cplusplus
inline
#else
static
#endif
	float
	vec3_dot(const float *a, const float *b)
{
	return (a && b) ? (a[0] * b[0] + a[1] * b[1] + a[2] * b[2]) : 0.0f;
}

#ifdef __cplusplus
inline
#else
static
#endif
	void
	vec3_normalize(const float *v, float *result)
{
	if (v && result)
	{
		float len = vec3_length(v);
		if (len < EPSILON)
		{
			result[0] = 0.0f;
			result[1] = 0.0f;
			result[2] = 1.0f; /* Valve's convention */
		}
		else
		{
			len = 1.0f / len;
			result[0] = v[0] * len;
			result[1] = v[1] * len;
			result[2] = v[2] * len;
		}
	}
}

#ifdef __cplusplus
inline
#else
static
#endif
	void
	vec3_copy(const float *src, float *dst)
{
	if (src && dst)
	{
		dst[0] = src[0];
		dst[1] = src[1];
		dst[2] = src[2];
	}
}

#ifdef __cplusplus
inline
#else
static
#endif
	int
	vec3_compare(const float *a, const float *b, float epsilon)
{
	if (!a || !b)
		return 0;
	return (fabs(a[0] - b[0]) < epsilon &&
			fabs(a[1] - b[1]) < epsilon &&
			fabs(a[2] - b[2]) < epsilon)
			   ? 1
			   : 0;
}

#ifdef __cplusplus
inline
#else
static
#endif
	void
	vec3_lerp(const float *a, const float *b, float t, float *result)
{
	if (a && b && result)
	{
		float clampedT = (t < 0.0f) ? 0.0f : (t > 1.0f) ? 1.0f
														: t;
		result[0] = a[0] + (b[0] - a[0]) * clampedT;
		result[1] = a[1] + (b[1] - a[1]) * clampedT;
		result[2] = a[2] + (b[2] - a[2]) * clampedT;
	}
}

#ifdef __cplusplus
inline
#else
static
#endif
	void
	vec3_clamp(const float *v, float min_val, float max_val, float *result)
{
	if (v && result)
	{
		result[0] = (v[0] < min_val) ? min_val : (v[0] > max_val) ? max_val
																  : v[0];
		result[1] = (v[1] < min_val) ? min_val : (v[1] > max_val) ? max_val
																  : v[1];
		result[2] = (v[2] < min_val) ? min_val : (v[2] > max_val) ? max_val
																  : v[2];
	}
}

#ifdef __cplusplus
inline
#else
static
#endif
	int
	vec3_is_zero(const float *v, float epsilon)
{
	if (!v)
		return 1;
	return (fabs(v[0]) < epsilon && fabs(v[1]) < epsilon && fabs(v[2]) < epsilon) ? 1 : 0;
}

#ifdef __cplusplus

/*=========================================================
 * 2DVector - C++ class wrapper
 *=========================================================*/
class Vector2D
{
public:
	/* Constructors */
	Vector2D(void) : x(0.0f), y(0.0f) {}
	Vector2D(float X, float Y) : x(X), y(Y) {}
	Vector2D(double X, double Y) : x(static_cast<float>(X)), y(static_cast<float>(Y)) {}
	Vector2D(int X, int Y) : x(static_cast<float>(X)), y(static_cast<float>(Y)) {}
	Vector2D(const Vector2D &v) : x(v.x), y(v.y) {}
	Vector2D(const float rgfl[2]) : x(rgfl[0]), y(rgfl[1]) {}

	/* Assignment operator */
	Vector2D &operator=(const Vector2D &v)
	{
		if (this != &v)
		{
			x = v.x;
			y = v.y;
		}
		return *this;
	}

	/* Arithmetic operators */
	Vector2D operator+(const Vector2D &v) const { return Vector2D(x + v.x, y + v.y); }
	Vector2D operator-(const Vector2D &v) const { return Vector2D(x - v.x, y - v.y); }
	Vector2D operator*(float fl) const { return Vector2D(x * fl, y * fl); }
	Vector2D operator/(float fl) const
	{
		if (fabs(fl) < EPSILON)
			return Vector2D(0.0f, 0.0f);
		return Vector2D(x / fl, y / fl);
	}
	Vector2D operator-() const { return Vector2D(-x, -y); }

	/* Compound assignment operators */
	Vector2D &operator+=(const Vector2D &v)
	{
		x += v.x;
		y += v.y;
		return *this;
	}
	Vector2D &operator-=(const Vector2D &v)
	{
		x -= v.x;
		y -= v.y;
		return *this;
	}
	Vector2D &operator*=(float fl)
	{
		x *= fl;
		y *= fl;
		return *this;
	}
	Vector2D &operator/=(float fl)
	{
		if (fabs(fl) >= EPSILON)
		{
			x /= fl;
			y /= fl;
		}
		else
		{
			x = y = 0.0f;
		}
		return *this;
	}

	/* Comparison operators with epsilon tolerance */
	bool operator==(const Vector2D &v) const
	{
		return (fabs(x - v.x) < EPSILON && fabs(y - v.y) < EPSILON);
	}
	bool operator!=(const Vector2D &v) const { return !(*this == v); }

	/* Type conversion operators */
	operator float *() { return &x; }
	operator const float *() const { return &x; }

	/* Methods */
	float Length(void) const
	{
		return static_cast<float>(sqrt(static_cast<double>(x * x + y * y)));
	}

	float LengthSquared(void) const { return x * x + y * y; }

	bool IsZero(float epsilon = EPSILON) const
	{
		return (fabs(x) < epsilon && fabs(y) < epsilon);
	}

	Vector2D Normalize(void) const
	{
		float flLen = Length();
		if (flLen < EPSILON)
		{
			return Vector2D(0.0f, 0.0f);
		}
		flLen = 1.0f / flLen;
		return Vector2D(x * flLen, y * flLen);
	}

	float DotProduct(const Vector2D &v) const { return x * v.x + y * v.y; }

	Vector2D Lerp(const Vector2D &target, float t) const
	{
		float clampedT = (t < 0.0f) ? 0.0f : (t > 1.0f) ? 1.0f
														: t;
		return Vector2D(
			x + (target.x - x) * clampedT,
			y + (target.y - y) * clampedT);
	}

	Vector2D Clamp(float min_val, float max_val) const
	{
		return Vector2D(
			(x < min_val) ? min_val : (x > max_val) ? max_val
													: x,
			(y < min_val) ? min_val : (y > max_val) ? max_val
													: y);
	}

	void CopyToArray(float *rgfl) const
	{
		if (rgfl)
		{
			rgfl[0] = x;
			rgfl[1] = y;
		}
	}

	void ToVec2(float out[2]) const
	{
		if (out)
		{
			out[0] = x;
			out[1] = y;
		}
	}

	static Vector2D FromVec2(const float in[2])
	{
		return in ? Vector2D(in[0], in[1]) : Vector2D(0.0f, 0.0f);
	}

	/* Data members */
	vec_t x, y;
};

/* Global 2D vector operators */
inline Vector2D operator*(float fl, const Vector2D &v) { return v * fl; }
inline float DotProduct(const Vector2D &a, const Vector2D &b) { return a.x * b.x + a.y * b.y; }

/*=========================================================
 * 3D Vector - Main vector class
 *=========================================================*/
class Vector
{
public:
	/* Constructors */
	Vector(void) : x(0.0f), y(0.0f), z(0.0f) {}
	Vector(float X, float Y, float Z) : x(X), y(Y), z(Z) {}
	Vector(double X, double Y, double Z) : x(static_cast<float>(X)), y(static_cast<float>(Y)), z(static_cast<float>(Z)) {}
	Vector(int X, int Y, int Z) : x(static_cast<float>(X)), y(static_cast<float>(Y)), z(static_cast<float>(Z)) {}
	Vector(const Vector &v) : x(v.x), y(v.y), z(v.z) {}
	Vector(const float rgfl[3]) : x(rgfl ? rgfl[0] : 0.0f), y(rgfl ? rgfl[1] : 0.0f), z(rgfl ? rgfl[2] : 0.0f) {}

	/* Assignment operator */
	Vector &operator=(const Vector &v)
	{
		if (this != &v)
		{
			x = v.x;
			y = v.y;
			z = v.z;
		}
		return *this;
	}

	/* Array access operators */
	float &operator[](int i) { return (&x)[i]; }
	const float &operator[](int i) const { return (&x)[i]; }

	/* Arithmetic operators */
	Vector operator+(const Vector &v) const { return Vector(x + v.x, y + v.y, z + v.z); }
	Vector operator-(const Vector &v) const { return Vector(x - v.x, y - v.y, z - v.z); }
	Vector operator*(float fl) const { return Vector(x * fl, y * fl, z * fl); }
	Vector operator/(float fl) const
	{
		if (fabs(fl) < EPSILON)
			return Vector(0.0f, 0.0f, 0.0f);
		return Vector(x / fl, y / fl, z / fl);
	}
	Vector operator-() const { return Vector(-x, -y, -z); }

	/* Compound assignment operators */
	Vector &operator+=(const Vector &v)
	{
		x += v.x;
		y += v.y;
		z += v.z;
		return *this;
	}
	Vector &operator-=(const Vector &v)
	{
		x -= v.x;
		y -= v.y;
		z -= v.z;
		return *this;
	}
	Vector &operator*=(float fl)
	{
		x *= fl;
		y *= fl;
		z *= fl;
		return *this;
	}
	Vector &operator/=(float fl)
	{
		if (fabs(fl) >= EPSILON)
		{
			x /= fl;
			y /= fl;
			z /= fl;
		}
		else
		{
			x = y = z = 0.0f;
		}
		return *this;
	}

	/* Comparison operators with epsilon tolerance */
	bool operator==(const Vector &v) const
	{
		return (fabs(x - v.x) < EPSILON && fabs(y - v.y) < EPSILON && fabs(z - v.z) < EPSILON);
	}
	bool operator!=(const Vector &v) const { return !(*this == v); }

	/* Type conversion operators */
	operator float *() { return &x; }
	operator const float *() const { return &x; }

	/* Methods */
	float Length(void) const
	{
		return static_cast<float>(sqrt(static_cast<double>(x * x + y * y + z * z)));
	}

	float LengthSquared(void) const { return x * x + y * y + z * z; }

	float Length2D(void) const
	{
		return static_cast<float>(sqrt(static_cast<double>(x * x + y * y)));
	}

	float Length2DSquared(void) const { return x * x + y * y; }

	bool IsZero(float epsilon = EPSILON) const
	{
		return (fabs(x) < epsilon && fabs(y) < epsilon && fabs(z) < epsilon);
	}

	Vector Normalize(void) const
	{
		float flLen = Length();
		if (flLen < EPSILON)
		{
			return Vector(0.0f, 0.0f, 1.0f); /* Valve's convention */
		}
		flLen = 1.0f / flLen;
		return Vector(x * flLen, y * flLen, z * flLen);
	}

	float DotProduct(const Vector &v) const { return x * v.x + y * v.y + z * v.z; }

	Vector CrossProduct(const Vector &v) const
	{
		return Vector(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);
	}

	Vector2D Make2D(void) const { return Vector2D(x, y); }

	Vector Lerp(const Vector &target, float t) const
	{
		float clampedT = (t < 0.0f) ? 0.0f : (t > 1.0f) ? 1.0f
														: t;
		return Vector(
			x + (target.x - x) * clampedT,
			y + (target.y - y) * clampedT,
			z + (target.z - z) * clampedT);
	}

	Vector Clamp(float min_val, float max_val) const
	{
		return Vector(
			(x < min_val) ? min_val : (x > max_val) ? max_val
													: x,
			(y < min_val) ? min_val : (y > max_val) ? max_val
													: y,
			(z < min_val) ? min_val : (z > max_val) ? max_val
													: z);
	}

	void CopyToArray(float *rgfl) const
	{
		if (rgfl)
		{
			rgfl[0] = x;
			rgfl[1] = y;
			rgfl[2] = z;
		}
	}

	void ToVec3(float out[3]) const
	{
		if (out)
		{
			out[0] = x;
			out[1] = y;
			out[2] = z;
		}
	}

	static Vector FromVec3(const float in[3])
	{
		return in ? Vector(in[0], in[1], in[2]) : Vector(0.0f, 0.0f, 0.0f);
	}

	void SetFromVec3(const float in[3])
	{
		if (in)
		{
			x = in[0];
			y = in[1];
			z = in[2];
		}
		else
		{
			x = y = z = 0.0f;
		}
	}

	static Vector Zero(void) { return Vector(0.0f, 0.0f, 0.0f); }
	static Vector One(void) { return Vector(1.0f, 1.0f, 1.0f); }
	static Vector Forward(void) { return Vector(1.0f, 0.0f, 0.0f); }
	static Vector Right(void) { return Vector(0.0f, 1.0f, 0.0f); }
	static Vector Up(void) { return Vector(0.0f, 0.0f, 1.0f); }

	/* Data members */
	vec_t x, y, z;
};

/* Global 3D vector operators */
inline Vector operator*(float fl, const Vector &v) { return v * fl; }
inline float DotProduct(const Vector &a, const Vector &b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vector CrossProduct(const Vector &a, const Vector &b) { return Vector(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x); }

/*=========================================================
 * Vector3 - Alias for compatibility (AFTER Vector class definition)
 *=========================================================*/
typedef Vector Vector3;

#else /* C89 mode */

/*=========================================================
 * C89 Vector structures
 *=========================================================*/
typedef struct Vector2D
{
	vec_t x, y;
} Vector2D;

typedef struct Vector
{
	vec_t x, y, z;
} Vector;

/* Vector3 is already forward declared as typedef struct Vector Vector3 */

#endif /* __cplusplus */

/* Common type definitions */
#define vec3_t Vector

#endif /* UTIL_VECTOR_H */