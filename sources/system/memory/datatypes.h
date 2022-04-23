//==============================================================================
//
//    OPENROX   : File datatypes.h
//
//    Contents  : Types definition
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_DATATYPES__
#define __OPENROX_DATATYPES__

#if defined(_MSC_VER) && (_MSC_VER < 1600)
#ifndef _STDINT
#include "pstdint.h"
#endif
#else
#include <stdint.h>
#include <stddef.h>
#endif

#define _INTPTR_T
#define _UINTPTR_T
#define _INTMAX_T
#define _UINTMAX_T

#include <system/arch/compiler.h>
#include <system/arch/platform.h>
#include <generated/config.h>

// A list of types which should be checked by cmake
typedef double         Rox_Double;
typedef float          Rox_Float;
typedef signed char    Rox_Schar;   // Signed char 
typedef char           Rox_Char;    // char (same as signed char)
typedef unsigned char  Rox_Uchar;   // Unsigned char
typedef unsigned short Rox_Ushort;
typedef signed int     Rox_Sint;
typedef unsigned int   Rox_Uint;
typedef int64_t        Rox_Slint;   // Signed long int
typedef uint64_t       Rox_Ulint;   // Unsigned long int
typedef unsigned char  Rox_Bool;
typedef void           Rox_Void;
typedef signed short   Rox_Sshort;
typedef size_t         Rox_Size;
typedef int            Rox_ErrorCode;
typedef Rox_ErrorCode  Rox_Error;
typedef char *         Rox_Ptr;
typedef int64_t        Rox_Int64;

//! Complex number 
struct Rox_Complex_Struct
{
  //! The real part 
  double real;

  //! The imaginary part 
  double imag;
};

typedef struct Rox_Complex_Struct Rox_Complex_Struct;

typedef struct Rox_Complex_Struct * Rox_Complex;

//! Dimension 2D in integer 
struct Rox_Dim_Int_Struct
{
  //! The width 
  int width;

  //! The height 
  int height;
};

typedef struct Rox_Dim_Int_Struct Rox_Dim_Int_Struct;

typedef struct Rox_Dim_Int_Struct * Rox_Dim_Int;

//! Dimension 2D in real
struct Rox_Dim_Real_Struct
{
  //! The width 
  double width;

  //! The height 
  double height;
};

typedef struct Rox_Dim_Real_Struct Rox_Dim_Real_Struct;

typedef struct Rox_Dim_Real_Struct * Rox_Dim_Real;

//! Sparse matrix cell 
struct Rox_Sparse_Value_Struct
{
  //! U coord 
  Rox_Uint u;
  //! v coord 
  Rox_Uint v;
  //! value 
  Rox_Double value;
};

typedef struct Rox_Sparse_Value_Struct Rox_Sparse_Value_Struct;

typedef struct Rox_Sparse_Value_Struct * Rox_Sparse_Value;

//!   A list of predefined type descriptions :
//!   A type description may be passed to untyped functions
//!   the first bit (least important) says if the type is floating point value
//!   The second bit states if the valu//!e is signed or unsigned
//!   The other bits defines the size of the type in bytes
//!   E.g. : signed int is 4 bytes, signed, not float : (4 << 2 + 1 << 1  + 0 == 18)
//!   Use this with strict caution
#define ROX_DATATYPES_ENUM_VALUES \
    X(ROX_TYPE_UCHAR  , 4)  \
    X(ROX_TYPE_SCHAR  , 6)  \
    X(ROX_TYPE_USHORT , 8)  \
    X(ROX_TYPE_SSHORT , 10)  \
    X(ROX_TYPE_UINT   , 16)  \
    X(ROX_TYPE_FLOAT  , 17)  \
    X(ROX_TYPE_SINT   , 18)  \
    X(ROX_TYPE_ULONG  , 32)  \
    X(ROX_TYPE_SLINT  , 34)  \
    X(ROX_TYPE_DOUBLE , 33)  

//! Preprocessor builds enum for us 
#define X(a, b) a = b,
enum Rox_Datatype_Description_Enum
{
   ROX_DATATYPES_ENUM_VALUES
};
typedef enum Rox_Datatype_Description_Enum Rox_Datatype_Description;
#undef X

// MACROS for datatype description 

// Is this described type a floating point ? 
#define ROX_DATATYPE_IS_FLOATING_POINT(X) (X&1)
// Is this described type a signed value ? 
#define ROX_DATATYPE_IS_SIGNED(X) (X&2)
// What is the size of this described type? 
#define ROX_DATATYPE_BYTECOUNT(X) (X>>2)
// Build a custom description using parameters 
#define ROX_DATATYPE_DESCRIPTION_CUSTOM(SIGNED,FLOAT,SIZE) ((Rox_Datatype_Description)((FLOAT)//((SIGNED)<<1)//((SIZE)<<2)))

#endif //__OPENROX_DATATYPES__
