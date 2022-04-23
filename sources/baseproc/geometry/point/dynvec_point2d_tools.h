//==============================================================================
//
//    OPENROX   : File dynvec_points2d_tools.h
//
//    Contents  : API of dynvec points2d tools module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_DYNVEC_POINTS2D_TOOLS__
#define __OPENROX_DYNVEC_POINTS2D_TOOLS__

#include <generated/dynvec_point2d_double.h>
#include <generated/dynvec_point2d_float.h>

#include <baseproc/geometry/point/points_struct.h>
#include <baseproc/geometry/point/point2d_struct.h>
#include <baseproc/maths/linalg/matrix.h>

//! \addtogroup Point2D
//! @{

//!
//! \param  [out]  points2D
//! \param  [in ]  data_points2D
//! \param  [in ]  numb_points2D
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_dynvec_point2d_double_set_data (
   Rox_DynVec_Point2D_Double points2D, 
   Rox_Double * data_points2D, 
   Rox_Uint numb_points2D
);

//!
//! \param  [out]  points2D
//! \param  [in ]  data_points2D
//! \param  [in ]  numb_points2D
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_dynvec_point2d_float_set_data (
   Rox_DynVec_Point2D_Float points2D, 
   Rox_Float * data_points2D, 
   Rox_Uint numb_points2D
);

//!
//! \param  [out]  points2D
//! \param  [in ]  points2D_struct
//! \param  [in ]  numb_points2D
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_dynvec_point2d_double_append_vector_point2d_double (
   Rox_DynVec_Point2D_Double dynvec_points2D, 
   Rox_Point2D_Double points2D, 
   Rox_Uint numb_points2D
);

//!
//! \param  [out]  points2D
//! \param  [in ]  points2D_struct_double
//! \param  [in ]  numb_points2D
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_dynvec_point2d_float_set_vector_point2d_double (
   Rox_DynVec_Point2D_Float points2D, 
   Rox_Point2D_Double_Struct * points2D_struct_double, 
   Rox_Uint numb_points2D
);

//! Apply generic homogeneous transformation
//! \param  [out]  result     points transformed
//! \param  [in ]  source     points to be transformed
//! \param  [in ]  A          3x3 transformation matrix
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_dynvec_point2d_double_transform_homogeneous ( 
   Rox_DynVec_Point2D_Double result,
   Rox_DynVec_Point2D_Double source,
   Rox_Matrix             A 
);

//! Convert pixel coordinates to normalized coordinates ( point_nor = inv(K) * point_pix )
//! \param  [out]  point_nor     points in normalized coordinates
//! \param  [in ]  point_pix     points in pixel coordinates
//! \param  [in ]  K             Camera calibration matrix
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_dynvec_point2d_convert_pixel_double_to_meter_double (
   Rox_DynVec_Point2D_Double point_nor, 
   Rox_DynVec_Point2D_Double point_pix, 
   Rox_Matrix                        K
);


//! Differences between two dynvec
//! \param  [out]  error
//! \param  [in ]  one
//! \param  [in ]  two
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_array2d_point2d_double_difference_point2d_float (
   Rox_Array2D_Double error, 
   Rox_DynVec_Point2D_Float one, 
   Rox_DynVec_Point2D_Float two
);


//! Differences between two dynvec
//! \param  [out] error
//! \param  [in ] one
//! \param  [in ] two
//! \return An error code
//! \todo To be tested
ROX_API Rox_ErrorCode rox_array2d_point2d_double_difference_point2d_double (
   Rox_Array2D_Double error, 
   Rox_DynVec_Point2D_Double one, 
   Rox_DynVec_Point2D_Double two
);

//! Distances between two dynvec
//! \param  [out]  dist
//! \param  [in ]  one
//! \param  [in ]  two
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_array2d_point2d_double_distance_point2d_float (
   Rox_Array2D_Double dist, 
   Rox_DynVec_Point2D_Float one, 
   Rox_DynVec_Point2D_Float two
);

//! Distances between two dynvec
//! \param  [out]  dist
//! \param  [in ]  one
//! \param  [in ]  two
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_array2d_point2d_double_distance_point2d_double (
   Rox_Array2D_Double dist, 
   Rox_DynVec_Point2D_Double one, 
   Rox_DynVec_Point2D_Double two
);


//! @}

#endif
