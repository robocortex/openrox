//==============================================================================
//
//    OPENROX   : File distance_point_to_point.h
//
//    Contents  : 
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_DISTANCE_POINT_TO_POINT__
#define __OPENROX_DISTANCE_POINT_TO_POINT__

#include <generated/array2d_double.h>
#include <generated/dynvec_point2d_double.h>

#include <baseproc/geometry/point/point2d.h>

//! Compute distance between a line and a point in a 2D plane
//! \param  [out]  distance       The distance
//! \param  [in ]  point2d_1      The 2D point1
//! \param  [in ]  point2d_2      The 2D point2
//! \return An error code
ROX_API Rox_ErrorCode rox_distance_point2d_to_point2d (
   Rox_Double * distance, 
   const Rox_Point2D_Double point2d_1, 
   const Rox_Point2D_Double point2d_2
);

ROX_API Rox_ErrorCode rox_dynvec_point2d_double_minimum_distance (
   Rox_Double * distance_min, 
   Rox_Uint * index, 
   Rox_Point2D_Double points2D_reprojected, 
   Rox_DynVec_Point2D_Double points2D_measured
);


#endif //__OPENROX_DISTANCE_POINT_TO_POINT__

