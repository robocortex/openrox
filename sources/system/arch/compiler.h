//============================================================================
//
//    OPENROX   : File compiler.h
//
//    Contents  : API of compiler module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//============================================================================

//!Define compiler specific information such as function attributes
//!for code portability

//! \addtogroup compiler
//! @{

//!Inlining function attribute
#ifndef ROX_INLINE
   #ifndef DEBUG
      #if defined(_MSC_VER)
         #define ROX_INLINE inline
      #elif defined(__GNUC__)
         #define ROX_INLINE __inline
      #endif   //_MSC_VER or __GNUC__
   #else
      #define ROX_INLINE
   #endif   //DEBUG
#endif   //ROX_INLINE

//!Force definition of mathematic constants
#if defined(_MSC_VER)
   #define _USE_MATH_DEFINES
#endif

//!Declare attribute for function external visibility in dll
#ifndef ROX_EXPORT
   #if defined ROX_EXPORT_SHARED
      #if defined(_MSC_VER)
         #define ROX_EXPORT __declspec(dllexport)
      #elif (defined __GNUC__) && (__GNUC__ >= 4)
         #define ROX_EXPORT __attribute__((visibility("default")))
      #else
         #define ROX_EXPORT
      #endif
   #elif defined ROX_EXPORT_STATIC
      #define ROX_EXPORT
   #endif   //ROX_EXPORT_SHARED or STATIC
#endif   //ROX_EXPORT

//!Declare that this variable is statically aligned on an adress boundary
#ifndef ROX_STATIC_ALIGN
   #if defined(_MSC_VER)
      #define ROX_STATIC_ALIGN(X) __declspec(align(X))
   #elif defined(__GNUC__)
      #define ROX_STATIC_ALIGN(X) __attribute__ ((aligned(X)))
   #else
      error
   #endif
#endif

//!Declare attribute for C++ inclusion
#ifndef ROX_EXTERN_C
   #ifdef __cplusplus
      #define ROX_EXTERN_C extern "C"
   #else
      #define ROX_EXTERN_C
   #endif
#endif

//! Declare attribute for function external visibility in dll and C++ inclusion
#ifndef ROX_EXPORT_C
   #if defined(_MSC_VER)
      #define ROX_EXPORT_C ROX_EXTERN_C __declspec(dllexport)
   #elif (defined __GNUC__) && (__GNUC__ >= 4)
      #define ROX_EXPORT_C ROX_EXTERN_C __attribute__((visibility("default")))
   #else
      #define ROX_EXPORT_C ROX_EXTERN_C
   #endif
#endif


//! @}

