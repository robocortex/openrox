//==============================================================================
//
//    OPENROX   : File connectivity.h
//
//    Contents  : API of connectivity module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_CONNECTIVITY__
#define __OPENROX_CONNECTIVITY__

#include <generated/objset_dynvec_point2d_sint.h>
#include <generated/array2d_uchar.h>
#include <generated/array2d_uint.h>

//! \ingroup Image
//! \addtogroup Connectivity
//! @{

//! Compute the list of connected chains
//! \param  [out]  lists          A list of vectors of coordinates
//! \param  [in ]  source         A binary image. This image will be modified by the function and may be empty after this function termination.
//! \param  [in ]  min_length     Minimum length of a list of pixel we want to be stored
//! \return An error code
ROX_API Rox_ErrorCode rox_array2d_uchar_connectivity (
   Rox_ObjSet_DynVec_Point2D_Sint lists, 
   Rox_Array2D_Uchar source, 
   Rox_Uint min_length
);

//! Compute the list of connected chains with same value
//! \param  [out]  lists          A list of vectors of coordinates
//! \param  [in ]  source         A integer image. This image will be modified by the function and may be empty after this function termination.
//! \param  [in ]  min_length     Minimum length of a list of pixel we want to be stored
//! \return An error code
ROX_API Rox_ErrorCode rox_array2d_uint_connectivity_value (
   Rox_ObjSet_DynVec_Point2D_Sint lists, 
   Rox_Array2D_Uint source, 
   Rox_Uint min_length
);

//! @}

#endif // __OPENROX_CONNECTIVITY__
