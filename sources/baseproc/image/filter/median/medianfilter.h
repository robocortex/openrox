//==============================================================================
//
//    OPENROX   : File medianfilter.h
//
//    Contents  : API of medianfilter module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_FILTER_MEDIAN__
#define __OPENROX_FILTER_MEDIAN__

#include <system/errors/errors.h>
#include <baseproc/image/image.h>

//! \ingroup Filtering
//! \addtogroup Median
//! @{

//! A median 2D filter
//! \param  [out]  dest           Convolved result
//! \param  [in ]  source         The image to filter
//! \param  [in ]  radius         Filter radius
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_image_filter_median (
   Rox_Image dest, 
   const Rox_Image source, 
   const Rox_Sint radius
);

//! @}  

#endif

