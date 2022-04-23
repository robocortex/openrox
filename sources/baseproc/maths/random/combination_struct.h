//==============================================================================
//
//    OPENROX   : File combination_struct.h
//
//    Contents  : API of combination module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_COMBINATION_STRUCT__
#define __OPENROX_COMBINATION_STRUCT__

#include <generated/dynvec_uint.h>

//! \ingroup Maths
//! \defgroup Statistics Statistics

//! \ingroup Statistics
//! \addtogroup RANDOM
//! @{

//! A random combination drawer
struct Rox_Combination_Struct
{
   //! Combination size = k
   Rox_Uint combination_size;

   //! this can simple be an array since it can be fixed ?
   //! The item bag of size 1 x n
   Rox_DynVec_Uint bag;

   //! The result draw of size 1 x k
   Rox_DynVec_Uint draw;
};

//! @}

#endif
