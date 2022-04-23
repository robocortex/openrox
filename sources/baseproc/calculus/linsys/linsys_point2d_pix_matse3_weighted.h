//==============================================================================
//
//    OPENROX   : File interaction_point2d_pix_matse3_weighted.h
//
//    Contents  : API of interaction_point2d_pix_matse3_weighted module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_SE3POINTS_WEIGHTED_PIXELS_RIGHT_PREMUL__
#define __OPENROX_SE3POINTS_WEIGHTED_PIXELS_RIGHT_PREMUL__

#include <generated/array2d_double.h>
#include <generated/dynvec_point3d_float.h>
#include <generated/dynvec_point3d_double.h>
#include <baseproc/maths/linalg/matrix.h>
#include <baseproc/maths/linalg/matut3.h>
#include <baseproc/image/imask/imask.h>

//! \ingroup Jacobians
//! \defgroup se3jacobians se3jacobians
//! \brief Jacobians relative to the SE3 group.

//! \addtogroup se3jacobians
//! @{

//! Compute the jacobian for points wrt se3,alpha,beta
//! \param  [out]  LtL           the result Hessian Matrix  (J^t*J)
//! \param  [out]  Lte           the result projected vector (J^t*diff)
//! \param  [in ]  diff          the error vector
//! \param  [in ]  weight        the weight vector (for robust estimation)
//! \param  [in ]  meters        the points in meters which were used to compute the error
//! \param  [in ]  calib         calibration matrix
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_jacobian_se3_from_points_pixels_weighted_premul_float (
   Rox_Matrix LtL, 
   Rox_Matrix Lte, 
   Rox_Array2D_Double diff, 
   Rox_Array2D_Double weight, 
   Rox_DynVec_Point3D_Float meters, 
   Rox_MatUT3 calib
);

//! Compute the jacobian for points wrt se3,alpha,beta
//! \param  [out]  LtL           the result Hessian Matrix  (J^t*J)
//! \param  [out]  Lte           the result projected vector (J^t*diff)
//! \param  [in ]  diff          the error vector
//! \param  [in ]  weight        the weight vector (for robust estimation)
//! \param  [in ]  meters        the points in meters which were used to compute the error
//! \param  [in ]  calib         calibration matrix
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_jacobian_se3_from_points_pixels_weighted_premul_double (
   Rox_Array2D_Double LtL, 
   Rox_Array2D_Double Lte, 
   const Rox_Array2D_Double diff, 
   const Rox_Array2D_Double weight, 
   const Rox_DynVec_Point3D_Double meters, 
   const Rox_MatUT3 calib
);

//! @}

#endif
