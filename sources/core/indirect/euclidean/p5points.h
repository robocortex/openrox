//==============================================================================
//
//    OPENROX   : File p5points.h
//
//    Contents  : API of p5points module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_P5P__
#define __OPENROX_P5P__

#include <generated/array2d_double.h>
#include <baseproc/geometry/point/point2d.h>

//! Compute the relative pose between 5 projected points
//! \param  [out]  ref2D  The 5 reference points in meters, normalized
//! \param  [in ]  cur2D  The 5 current points in meters, normalized
//! \return An error code
ROX_API Rox_ErrorCode rox_pose_from_5_points (
   Rox_Point2D_Double ref2D, 
   Rox_Point2D_Double cur2D);

#endif
