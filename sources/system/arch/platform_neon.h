//==============================================================================
//
//    OPENROX   : File platform_neon.h
//
//    Contents  : API of platform_neon module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_PLATFORM_NEON__
#define __OPENROX_PLATFORM_NEON__

//! For neon instructions, memory is vector aligned and data is padded to be a modulo of 16 bytes
#define ROX_DEFAULT_ALIGNMENT 16
#define ROX_ROW_BYTECOUNT_MULTIPLIER 16

// This include file is mandatory to use neon intrinsics
#include <arm_neon.h>

//! Four float access
typedef union
{
 	//! To be commented
   float32x4_t ssetype;
 	//! To be commented
   float tab[4];
}
Rox_Neon_Float;

//! Four int access
typedef union
{
 	//! To be commented
   int32x4_t ssetype;
  	//! To be commented
  int tab[4];
}
Rox_Neon_Sint;

//! Four unsigned int access
typedef union
{
 	//! To be commented
   uint32x4_t ssetype;
  	//! To be commented
  unsigned int tab[4];
}
Rox_Neon_Uint;

//! Four unsigned short access
typedef union
{
  	//! To be commented
  uint16x4_t ssetype;
  	//! To be commented
  unsigned short int tab[4];
}
Rox_Neon_Ushort_half;

#endif // __OPENROX_ROX_PLATFORM_NEON__
