//==============================================================================
//
//    OPENROX   : File remapbilinear_onepixel.h
//
//    Contents  : API of remapbilinear_onepixel module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_REMAPBILINEAR_ONEPIXEL__
#define __OPENROX_REMAPBILINEAR_ONEPIXEL__

#include <baseproc/geometry/point/point2d.h>
#include <baseproc/image/image.h>

//! \ingroup Image
//! \addtogroup Remap
//! @{

//! Given an input array, interpolate a pixel at given location
//! \param  [out]  output         The result value (if the location is out of range, returns an error)
//! \param  [in ]  input          The input array
//! \param  [in ]  pos            2d location coordinates
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_remap_onepixel_bilinear_uchar (
   Rox_Uchar * output, 
   const Rox_Image input, 
   const Rox_Point2D_Float pos
);

//! @}

#endif
