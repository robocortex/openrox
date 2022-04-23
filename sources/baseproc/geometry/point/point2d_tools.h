//==============================================================================
//
//    OPENROX   : File points2d_tools.h
//
//    Contents  : API of points3d tools module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_POINTS2D_TOOLS__
#define __OPENROX_POINTS2D_TOOLS__

#include <generated/dynvec_point2d_double.h>
#include <generated/dynvec_point2d_float.h>

#include <baseproc/geometry/point/points_struct.h>
#include <baseproc/geometry/point/point2d_struct.h>
#include <baseproc/maths/linalg/matut3.h>


//! \addtogroup Point2D
//! @{

//! Copy
//! \param  [out]  output
//! \param  [in ]  input
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_point2d_double_copy (
   Rox_Point2D_Double output, 
   const Rox_Point2D_Double input
);

//! Convert
//! \param  [out]  output
//! \param  [in ]  input
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_point2d_float_convert_double (
   Rox_Point2D_Double output, 
   Rox_Point2D_Float input
);

//! Convert
//! \param  [out]  point_nor
//! \param  [in ]  point_pix
//! \param  [in ]  K
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_point2d_convert_pixel_float_to_meter_float (
   Rox_Point2D_Float point_nor, 
   const Rox_Point2D_Float point_pix, 
   const Rox_MatUT3 K
);

//! Transform point in pixels to meters coordinates
//! point_nor = inv( pix_K_met ) * point_pix
//! \param  [out]  point_nor
//! \param  [in ]  point_pix
//! \param  [in ]  pix_K_met
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_point2d_convert_pixel_double_to_meter_double (
   Rox_Point2D_Double point_nor, 
   const Rox_Point2D_Double point_pix, 
   const Rox_MatUT3 pix_K_met
);


ROX_API Rox_ErrorCode rox_point2d_convert_pixel_float_to_meter_double (
   Rox_Point2D_Double point_nor, 
   const Rox_Point2D_Float point_pix, 
   const Rox_MatUT3 K
);

ROX_API Rox_ErrorCode rox_point2d_convert_meter_double_to_pixel_float(
   Rox_Point2D_Float point_pix, 
   const Rox_Point2D_Double point_nor, 
   const Rox_MatUT3 K
);

ROX_API Rox_ErrorCode rox_point2d_float_distance (
   Rox_Float * distance, 
   const Rox_Point2D_Float point2d_1, 
   const Rox_Point2D_Float point2d_2 
);

ROX_API Rox_ErrorCode rox_point2d_double_distance (
   Rox_Double * distance, 
   const Rox_Point2D_Double point2d_1, 
   const Rox_Point2D_Double point2d_2
);

//! Compute the distance between two 3D points
//! \param  [out]  bounding_box   The 8 corners of the bounding box
//! \param  [in ]  points_list    The list of 3D points [u1, v1, u2, v2, ..., un, vn ]  
//! \param  [in ]  nb_points      The number of points n in point_list 
//! \return An error code
ROX_API Rox_ErrorCode rox_point2d_double_compute_bounding_box ( 
   Rox_Point2D_Double bounding_box, 
   const Rox_Double * points_list,
   const Rox_Sint nb_points
);


ROX_API Rox_ErrorCode rox_point2d_double_compute_bounding_box ( 
   Rox_Point2D_Double bounding_box, 
   const Rox_Double * points_list,
   const Rox_Sint nb_points
);

ROX_API Rox_ErrorCode rox_vector_point2d_double_compute_bounding_box ( 
   Rox_Point2D_Double bounding_box, 
   const Rox_Point2D_Double points_list,
   const Rox_Sint nb_points
);

//! @}

#endif
