//==============================================================================
//
//    OPENROX   : File matsl3_from_n_points.h
//
//    Contents  : API of sl3fromNpoints module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_MATSL3_FROM_POINTS__
#define __OPENROX_MATSL3_FROM_POINTS__

#include <baseproc/geometry/point/point2d_struct.h>
#include <baseproc/geometry/point/point3d_struct.h>
#include <baseproc/geometry/point/point2d.h>
#include <baseproc/geometry/point/point3d.h>

#include <baseproc/maths/linalg/matsl3.h>

//! \ingroup MatSL3
//! \addtogroup sl3from4points
//! @{

//! Generate a homography given N>=4 points in 2D and the corresponding 2D reference points
//! \param  [out]  homography     Computed homography (cur_H_ref)
//! \param  [in ]  ref            The N 2D points in the reference view
//! \param  [in ]  cur            The N 2D points in the current view
//! \param  [in ]  nbpoints       The N value
//! \return An error code
ROX_API Rox_ErrorCode rox_matsl3_from_n_points_double (
   Rox_MatSL3 homography, 
   Rox_Point2D_Double ref, 
   Rox_Point2D_Double cur, 
   Rox_Uint nbpoints);

//! Generate a homography given N>=4 points in 2D and the corresponding 3D reference points
//! \param  [out]  homography     Computed homography (cur_H_ref)
//! \param  [in ]  ref            The N 3D points in the reference frame
//! \param  [in ]  cur            The N 2D points in the current view
//! \param  [in ]  nbpoints       The N value
//! \return An error code
ROX_API Rox_ErrorCode rox_matsl3_from_n_points3d_to_points2d_double(
   Rox_MatSL3 homography, 
   Rox_Point3D_Double ref, 
   Rox_Point2D_Double cur, 
   Rox_Uint nbpoints);

//! @}

#endif // __OPENROX_MATSL3_FROM_N_POINTS__
