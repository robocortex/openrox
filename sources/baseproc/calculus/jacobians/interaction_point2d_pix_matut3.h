//==============================================================================
//
//    OPENROX   : File interaction_matut3_point2d_pix.h
//
//    Contents  : API of interaction_matut3_point2d_pix module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_INTERACTION_MATUT3_POINT2D_PIX__
#define __OPENROX_INTERACTION_MATUT3_POINT2D_PIX__

#include <generated/array2d_double.h>
#include <baseproc/geometry/point/point2d.h>
#include <baseproc/maths/linalg/matrix.h>

//! \addtogroup Calibration_Jacobians
//! @{

//! Compute the Interaction matrix for camera calibration wrt q, cu, cv
//! \param  [out]  L             The interaction matrix
//! \param  [in ]  p             List of 2D points in pixels
//! \param  [in ]  cu            the u coordinate of the principal point (in pixels)
//! \param  [in ]  cv            the v coordinate of the principal point (in pixels)
//! \param  [in ]  count         length of the 2D point list
//! \param  [in ]  ddl           number of degree of freedom to estimate
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_jacobian_points_2d_campar (
   Rox_Matrix L, 
   const Rox_Point2D_Double p, 
   const Rox_Double cu, 
   const Rox_Double cv, 
   const Rox_Sint count, 
   const Rox_Sint ddl
);

//! Compute the jacobian for the calibration of fu wrt q, u0, v0
//! \param  [out]  L              The Jacobian
//! \param  [in ]  p              List of 2D points
//! \param  [in ]  cu             The optical center
//! \param  [in ]  cv             The optical center
//! \param  [in ]  count          Length of the 2D point list
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_jacobian_points_2d_campar_fu (
   Rox_Matrix L, 
   const Rox_Point2D_Double p, 
   const Rox_Double cu, 
   const Rox_Double cv, 
   const Rox_Sint count
);

//! Compute the jacobian for the calibration of fu and fv wrt q, u0, v0
//! \param  [out]  L              The Jacobian
//! \param  [in ]  p              List of 2D points
//! \param  [in ]  cu             The optical center
//! \param  [in ]  cv             The optical center
//! \param  [in ]  count          Length of the 2D point list
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_jacobian_points_2d_campar_fu_fv (
   Rox_Matrix L, 
   const Rox_Point2D_Double p, 
   const Rox_Double cu, 
   const Rox_Double cv, 
   const Rox_Sint count
);

//! Compute the jacobian for calibration of fu, cu and cv wrt q
//! \param  [out]  L              The Jacobian
//! \param  [in ]  p              List of 2D points
//! \param  [in ]  count          Length of the 2D point list
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_jacobian_points_2d_campar_fu_cu_cv (
   Rox_Matrix L, 
   const Rox_Point2D_Double p, 
   const Rox_Sint count
);

//! Compute the jacobian for calibration of fu, fv, cu and cv wrt q, u0, v0
//! \param  [out]  L              The Jacobian
//! \param  [in ]  p              List of 2D points
//! \param  [in ]  count          Length of the 2D point list
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_jacobian_points_2d_campar_fu_fv_cu_cv (
   Rox_Matrix L, 
   const Rox_Point2D_Double p, 
   const Rox_Sint count
);

//! Compute the jacobian for calibration of fu, fv, cu, cv and skew wrt q
//! \param  [out]  L              The Jacobian
//! \param  [in ]  p              Vector of 2D points
//! \param  [in ]  count          Length of the 2D point list
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_jacobian_points_2d_campar_fu_fv_cu_cv_s (
   Rox_Matrix L, 
   const Rox_Point2D_Double p, 
   const Rox_Sint count
);

//! Build the solution vector
//! \param  [out]  xK             The returned result vector
//! \param  [in ]  xK_ddl         The input vector
//! \param  [in ]  cu             The optical center
//! \param  [in ]  cv             The optical center
//! \param  [in ]  ddl            Number of degree of freedom to estimate
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_campar_ddl (
   Rox_Array2D_Double xK, 
   const Rox_Array2D_Double xK_ddl, 
   Rox_Double cu, 
   Rox_Double cv, 
   const Rox_Sint ddl
);

//! @}

#endif // __OPENROX_INTERACTION_MATUT3_POINT2D_PIX__
