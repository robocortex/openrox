//==============================================================================
//
//    OPENROX   : File ellipse_transform.h
//
//    Contents  : API of ellipse_transform module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_ELLIPSE_TRANSFORM__
#define __OPENROX_ELLIPSE_TRANSFORM__

#include <baseproc/geometry/ellipse/ellipse2d.h>
#include <baseproc/geometry/ellipse/ellipse3d.h>
#include <generated/array2d_double.h>

//! \addtogroup ellipse
//! @{

//! Transform a 2d ellipse a calibration matrix
//! \param  [out]  ellipse2d_out  The input 2d ellipse
//! \param  [in ]  ellipse2d_inp  The input 2d ellipse
//! \param  [in ]  matct2         The calibration matrix
//! \return An error code
ROX_API Rox_ErrorCode rox_ellipse2d_transform_pixels_to_meters(Rox_Ellipse2D ellipse2d_out, const Rox_Array2D_Double matct2, const Rox_Ellipse2D ellipse3d_inp);

//! Transform a 3d ellipse with planes coordinates given a pose
//! \param  [out]  ellipse3d_out  The input 3d ellipse
//! \param  [in ]  ellipse3d_inp  The input 3d ellipse
//! \param  [in ]  matse3         The tranformation pose
//! \return An error code
ROX_API Rox_ErrorCode rox_ellipse3d_transform(Rox_Ellipse3D ellipse3d_out, const Rox_Array2D_Double matse3, const Rox_Ellipse3D ellipse3d_inp);

//! @}

#endif
