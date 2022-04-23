//==============================================================================
//
//    OPENROX   : File line_from_points.h
//
//    Contents  : API of line_from_points module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_LINE_FROM_POINTS__
#define __OPENROX_LINE_FROM_POINTS__

#include <generated/dynvec_point2d_double.h>
#include <generated/dynvec_point2d_float.h>
#include <generated/dynvec_uint.h>

#include <baseproc/geometry/point/point3d.h>

#include <baseproc/geometry/point/point2d_struct.h>
#include <baseproc/geometry/point/point3d_struct.h>
#include <baseproc/geometry/line/line2d.h>
#include <baseproc/geometry/line/line3d.h>
#include <baseproc/geometry/segment/segment3d.h>
#include <system/errors/errors.h>


#include <baseproc/geometry/line/line2d_struct.h>

//! \addtogroup Line
//! @{

ROX_API Rox_ErrorCode rox_line3d_parametric_from_2_point3d (
   Rox_Line3D_Parametric line3d_parametric,
   const Rox_Point3D_Double pt1,
   const Rox_Point3D_Double pt2
);

//! Build a 3D planes line from two 3D points
//! \param  [out]  line3d_planes     	The computed 3d line in planes coordinates
//! \param  [in ]  point3d_1   			The first 3D point
//! \param  [in ]  point3d_2   			The second 3D point
//! \return An error code
ROX_API Rox_ErrorCode rox_line3d_planes_from_2_point3d (
   Rox_Line3D_Planes line3d_planes, 
   const Rox_Point3D_Double point3d_1, 
   const Rox_Point3D_Double point3d_2
);

//! Build a 3D planes line from two 3D points
//! \param  [out]  line3d_planes       The computed 3d line in planes coordinates
//! \param  [in ]  point3d_1           The first 3D point
//! \param  [in ]  point3d_2           The second 3D point
//! \return An error code
ROX_API Rox_ErrorCode rox_line3d_planes_from_2_point3d_float (
   Rox_Line3D_Planes line3d_planes, 
   const Rox_Point3D_Float pt1, 
   const Rox_Point3D_Float pt2
);

//! Build a 3D planes line from a 3D segment
//! \param  [out]  line3d_plucker   	The computed 3d line in planes coordinates
//! \param  [in ]  segment3d   			The 3d segment
//! \return An error code
ROX_API Rox_ErrorCode rox_line3d_planes_from_segment3d (
   Rox_Line3D_Planes line3d_planes, 
   const Rox_Segment3D segment3d
   );

//! Build a 3D plucker line from two 3D points
//! \param  [out]  line3d_plucker   	The computed 3d line in plucker coordinates
//! \param  [in ]  point3d_1   			The first 3D point
//! \param  [in ]  point3d_2   			The second 3D point
//! \return An error code
ROX_API Rox_ErrorCode rox_line3d_plucker_from_2_point3d (
   Rox_Line3D_Plucker line3d_plucker, 
   const Rox_Point3D_Double point3d_1, 
   const Rox_Point3D_Double point3d_2
);

//! \brief Build a 2D normal line from two 2D points
//! \param  [out]  line2d_normal   		The computed 2d line in normal form
//! \param  [in ]  point2d_1   			The first 2D point
//! \param  [in ]  point2d_2   			The second 2D point
//! \return An error code
ROX_API Rox_ErrorCode rox_line2d_normal_from_2_point2d (
   Rox_Line2D_Normal line2d_normal, 
   const Rox_Point2D_Double point2d_1, 
   const Rox_Point2D_Double point2d_2
);

//! \brief Build a 2D homogeneous line from two 2D points
//! \param  [out]  line2d_homogeneous   The computed 2d line in homogeneous form
//! \param  [in ]  point2d_1   			The first 2D point
//! \param  [in ]  point2d_2   			The second 2D point
//! \return An error code
ROX_API Rox_ErrorCode rox_line2d_homogeneous_from_2_point2d (
   Rox_Line2D_Homogeneous line2d_homogeneous, 
   const Rox_Point2D_Double point2d_1, 
   const Rox_Point2D_Double point2d_2
);

//! \brief Build a 2D polar line from two 2D points
//! \param [out]  line2d_homogeneous   The computed 2d line in homogeneous form
//! \param [in ]  points2d              The vector of 2D points
//! \return An error code
ROX_API Rox_ErrorCode rox_line2d_homogeneous_from_n_point2d (
   Rox_Line2D_Homogeneous line2d_homogeneous, 
   const Rox_DynVec_Point2D_Double points2d
);

//! \brief Build a 2D homogeneous line from rho and theta parameters l = [cos(theta); siz(theta); -rho];
//! \param  [out]  line2d_homogeneous    The computed 2d line in homogeneous form
//! \param  [in ]  rho                  The rho parameter
//! \param  [in ]  theta                The theta parameter
//! \return An error code
ROX_API Rox_ErrorCode rox_line2d_homogeneous_from_rho_theta (
   Rox_Line2D_Homogeneous line2d_homogeneous, 
   const Rox_Double rho, 
   const Rox_Double theta);

//! \brief Build a 2D homogeneous line from n points (using ransac draw)
//! \param  [out]  line2d_homogeneous    The computed 2d line in homogeneous form
//! \param  [in ]   points2D             The vector of 2D points
//! \param  [in ]   distance_threshold   Acceptable distance threshold
//! \return An error code
ROX_API Rox_ErrorCode rox_line2d_homogeneous_from_n_point2d_ransac(
   Rox_Line2D_Homogeneous line2d_homogeneous, 
   const Rox_DynVec_Point2D_Float points2D, 
   const Rox_Float distance_threshold);

// TODO: to be dispaced in inout/line/line2d_print.c
ROX_API Rox_ErrorCode rox_line2d_parmetric_print (
   Rox_Line2D_Homogeneous line2D
);


ROX_API Rox_ErrorCode rox_dynvec_point2d_float_select_from_indexes (
   Rox_DynVec_Point2D_Float points2D_inliers, 
   Rox_DynVec_Uint points2D_inliers_indexes, 
   Rox_DynVec_Point2D_Float points2D
);


//! @}

#endif
