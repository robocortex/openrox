//==============================================================================
//
//    OPENROX   : File p7p.h
//
//    Contents  : API of p7p module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_P7P_ODOMETRY__
#define __OPENROX_P7P_ODOMETRY__

#include <generated/array2d_double.h>
#include <baseproc/geometry/point/point2d.h>
#include <baseproc/geometry/point/point3d.h>

//! \ingroup Geometry
//! \addtogroup NonOverlap
//! @{

//! Compute a pose from 7 lines in multiple non overlapping cameras (3 cameras at least)
//! \param [out] pose the computed pose inverse (rTc)
//! \param [in] gac the current view line (Plucker representation, in first camera frame)
//! \param [in] gbc the current view line (Plucker representation, in first camera frame)
//! \param [in] gar the reference view line (Plucker representation, in first camera frame)
//! \param [in] gbr the reference view line (Plucker representation, in first camera frame)
//! \return An error code
ROX_API Rox_ErrorCode rox_odometry_nooverlap_make_p7l(Rox_Array2D_Double pose, Rox_Point2D_Double  gac, Rox_Point3D_Double gbc, Rox_Point2D_Double gar, Rox_Point3D_Double gbr);

//! Compute a pose from 7 lines in multiple non overlapping cameras
//! \param [out] pose the computed pose (cTr)
//! \param [in] cam1To the pose of the first camera wrt a common frame
//! \param [in] curcam1 the 3 current view observations in the first camera.
//! \param [in] refcam1 the 3 current view observations in the first camera.
//! \param [in] cam2To the pose of the second camera wrt a common frame
//! \param [in] curcam2 the 2 current view observations in the second camera.
//! \param [in] refcam2 the 2 current view observations in the second camera.
//! \param [in] cam3To the pose of the third camera wrt a common frame
//! \param [in] curcam3 the 2 current view observations in the third camera.
//! \param [in] refcam3 the 2 current view observations in the third camera.
//! \return An error code
ROX_API Rox_ErrorCode rox_odometry_nooverlap_make_p7p (Rox_Array2D_Double pose, Rox_Array2D_Double cam1To, Rox_Point2D_Double  curcam1, Rox_Point2D_Double  refcam1, Rox_Array2D_Double cam2To, Rox_Point2D_Double  curcam2, Rox_Point2D_Double  refcam2, Rox_Array2D_Double cam3To, Rox_Point2D_Double  curcam3, Rox_Point2D_Double  refcam3);

//! @}

#endif
