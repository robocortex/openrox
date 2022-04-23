//==============================================================================
//
//    OPENROX   : File pointsfromtemplate.h
//
//    Contents  : API of pointsfromtemplate module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_POINTS_FROM_TEMPLATE__
#define __OPENROX_POINTS_FROM_TEMPLATE__

#include <generated/array2d_double.h>
#include <generated/dynvec_point2d_float.h>
#include <generated/dynvec_point3d_float.h>

//! \ingroup Geometry
//! \addtogroup Point3D
//! @{

//! Given 2D points (in image coordinates), compute the "3D" points in meters (supposing plane Z=0)
//! \param  [out]  output         An array of 3D output points (Allocated externally)
//! \param  [in ]  input          An array of 2D input points
//! \param  [in ]  calib          Calibration
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_points3d_float_from_template (
   Rox_DynVec_Point3D_Float output, 
   Rox_DynVec_Point2D_Float input, 
   Rox_Array2D_Double calib
);

//! @}

#endif
