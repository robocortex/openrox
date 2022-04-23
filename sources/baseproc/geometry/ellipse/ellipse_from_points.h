//==============================================================================
//
//    OPENROX   : File ellipse_from_points.h
//
//    Contents  : API of ellipse_from_points module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_ELLIPSE_FROM_POINTS__
#define __OPENROX_ELLIPSE_FROM_POINTS__

#include <generated/dynvec_point2d_double.h>
#include <generated/dynvec_point2d_float.h>

#include <baseproc/geometry/point/point2d_struct.h>
#include <baseproc/geometry/point/point3d_struct.h>
#include <baseproc/geometry/ellipse/ellipse2d.h>
#include <baseproc/geometry/ellipse/ellipse3d.h>
#include <system/errors/errors.h>

//! \addtogroup Ellipse
//! @{

//! \brief Build a 2D parametric ellipse from n 2D points
//! \param  [out]  ellipse2d_parametric 	  The computed 2d line in parametric coordinates
//! \param  [in ]  points2d            	  The vector of 2D points
//! \return An error code
ROX_API Rox_ErrorCode rox_ellipse2d_parametric_from_n_point2d(Rox_Ellipse2D_Parametric ellipse2d_parametric, const Rox_DynVec_Point2D_Double points2d);

//! \brief Build a 2D parametric ellipse from n 2D points (using ransac draw)
//! \param  [out]  ellipse2d_parametric     The computed 2d line in parametric coordinates
//! \param  [in ]  points2D                 The vector of 2D points
//! \param  [in ]  distance_threshold       Acceptable distance threshold
//! \return An error code
ROX_API Rox_ErrorCode rox_ellipse2d_parametric_from_n_point2d_ransac(Rox_Ellipse2D_Parametric ellipse2d_parametric, const Rox_DynVec_Point2D_Float points2D, const Rox_Float distance_threshold);

//! @}

#endif
