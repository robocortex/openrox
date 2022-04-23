//============================================================================
//
//    OPENROX   : File maths_macros.h
//
//    Contents  : API of maths_macros module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//============================================================================

#ifndef __OPENROX_MATHS_MACROS__
#define __OPENROX_MATHS_MACROS__

#include <math.h>

#include <float.h>
#include <baseproc/maths/random/random.h>

//! Get minimal value
#define ROX_MIN(A,B) (((A)<(B))?(A):(B))

//! Get maximal value
#define ROX_MAX(A,B) (((A)>(B))?(A):(B))

//! Check if value is close to zero
#define ROX_IS_ZERO_DOUBLE(A) (fabs(A) < DBL_EPSILON)

//! Check if value is close to zero with given precision
#define ROX_IS_ZERO_PREC(A, PREC) (fabs(A) < PREC)

//! Pow single float
#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 199901L)
   #define ROX_POWF(A,B) powf(A,B)
#else
   #define ROX_POWF(A,B) pow(A,B)
#endif

//! Undefine Pi constants
#ifdef M_PI
   #undef M_PI
#endif

#ifdef M_PI_2
   #undef M_PI_2
#endif

#ifdef M_PI_4
   #undef M_PI_4
#endif

//! Define rox Pi constants
#define ROX_PI 3.14159265358979323846
#define ROX_2PI (2.0*ROX_PI)
#define ROX_PI_2 1.57079632679489661923
#define ROX_PI_4 0.78539816339744830962

//! Random value
#define ROX_RAND(MIN, MAX) ( ((double)MIN) + (((double)rox_rand()) / ((double)RAND_MAX)) * (double)(MAX - MIN))

//! Round to closest int
#define ROX_ROUND_DOUBLE(A) ((int)(A + ((A > 0.0f) ? 0.5 : -0.5)))

//! Round to closest int
#define ROX_ROUND_FLOAT(A) ((int)(A + ((A > 0.0f) ? 0.5f : -0.5f)))

//! SIGN of a floating point
#ifndef ROX_SIGN
#define ROX_SIGN(A) ((A<0.0)?-1.0:1.0)
#endif

//! sinc
#define sinc(X) (sin(ROX_PI * X) / (ROX_PI * X))

#endif
