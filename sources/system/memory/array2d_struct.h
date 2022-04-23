//==============================================================================
//
//    OPENROX   : File array2d_struct.h
//
//    Contents  : API of array2d module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_ARRAY2D_STRUCT__
#define __OPENROX_ARRAY2D_STRUCT__

#ifdef __cplusplus
extern "C" {
#endif

#include <system/memory/array_struct.h>

//! \ingroup System
//! @defgroup Array2D Array2D
//! @{

//! 2D array structure. MUST be used in place of Rox_Array even for a 1D vector
struct Rox_Array2D_Struct
{
   //! Number of cols
   Rox_Sint cols;

   //! Number of rows
   Rox_Sint rows;

   //! Number of bytes used per row (including padding)
   Rox_Sint step;

   //! Number of blocks
   Rox_Uint nbblocks;

   //! How many elements to shift before getting aligned adress
   Rox_Uint align_shift;

   //! Rox_Array containing stacked rows
   Rox_Array data;

   //! Row pointers
   Rox_Void ** rows_ptr;
   
   //! Block of rows pointers
   Rox_Void *** block_ptr;
};

//! @}

#ifdef __cplusplus
}
#endif

#endif
