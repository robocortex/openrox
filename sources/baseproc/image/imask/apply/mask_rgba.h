//==============================================================================
//
//    OPENROX   : File mask_rgba.h
//
//  	Contents  : API of mask_rgba module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//==============================================================================

#ifndef __OPENROX_MASK_RGBA__
#define __OPENROX_MASK_RGBA__

#include <generated/array2d_uchar.h>
#include <generated/array2d_uint.h>
#include <stdio.h>

//! \ingroup Mask
//! \brief Tools to apply masks on rgba image
//! @{
 
//! Sets the alpha value to 0 or unchanged according to the mask in a rgba array
//! \param  [out]  source 			 source and destination image
//! \param  [in ]  mask 			 mask to apply
//! \param  [in ]  value 			 value to apply on alpha channel
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_mask_alpha_roxrgba ( 
   Rox_Array2D_Uchar source, 
   const Rox_Array2D_Uint mask, 
   const Rox_Char value
);

//! Sets the alpha value to 0 or unchanged according to the mask in a rgba array
//! \param  [out]  source 			 source and destination image
//! \param  [in ]  mask 			 mask to apply
//! \param  [in ]  value 			 value to apply on alpha channel
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_mask_alpha_roxrgba_reverse ( 
   Rox_Array2D_Uchar source, 
   const Rox_Array2D_Uint mask, 
   const Rox_Char value 
);

//! @} 

#endif // __OPENROX_MASK_RGBA__
