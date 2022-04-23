//==============================================================================
//
//    OPENROX   : File roxrgba_to_roxlab.h
//
//    Contents  : API of roxrgba_to_roxlab module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_IMAGE_ROXRGBA_TO_ROXLAB__
#define __OPENROX_IMAGE_ROXRGBA_TO_ROXLAB__

#include <generated/array2d_uint.h>

//! \ingroup Image
//! \addtogroup Image_Conversion
//! @{

//! Convert a rgba rox internal image to lab rox internal image
//! \param  [out]  dest           destination image
//! \param  [in ]  source         source image
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_roxrgba_to_roxlab ( Rox_Array2D_Uint dest, const Rox_Array2D_Uint source );

//! @} 

#endif // __OPENROX_IMAGE_ROXRGBA_TO_ROXLAB__
