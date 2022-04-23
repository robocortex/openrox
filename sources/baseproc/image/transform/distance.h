//==============================================================================
//
//    OPENROX   : File distance.h
//
//    Contents  : API of distance module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_DISTANCE_TRANSFORM__
#define __OPENROX_DISTANCE_TRANSFORM__

#include <generated/array2d_sshort.h>
#include <generated/array2d_point2d_sshort.h>
#include <baseproc/image/image.h>

//! \ingroup Image
//! \addtogroup transform
//! @{

//! Compute the distance map of a binary image (Distance to the closest non zero value)
//! \param  [out] distancemap       Define for each pixel the distance to the closest valid point
//! \param  [in]	closestpoints     Define for each pixel the closest coordinates of a valid point
//! \param  [in]	source			   The source image
//! \return An error code
ROX_API Rox_ErrorCode rox_array2d_uchar_distancetransform(Rox_Array2D_Sshort distancemap, Rox_Array2D_Point2D_Sshort closestpoints, Rox_Image source);

//! @}

#endif
