//============================================================================
//
//    OPENROX   : File linsys_point2d_nor_matse3_weighted.h
//
//    Contents  : API of linsys_point2d_nor_matse3_weighted module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//============================================================================

#ifndef __OPENROX_SE3POINTS_WEIGHTED_RIGHT_PREMUL__
#define __OPENROX_SE3POINTS_WEIGHTED_RIGHT_PREMUL__

#include <generated/array2d_double.h>
#include <generated/dynvec_point2d_float.h>
#include <generated/dynvec_point3d_float.h>
#include <baseproc/maths/linalg/matrix.h>
#include <baseproc/image/imask/imask.h>

//! \ingroup Jacobians
//! \addtogroup se3jacobians
//! @{

//! Compute the jacobian for points wrt se3,alpha,beta
//! \param  [out]  LtL            The result Hessian Matrix  (J^t*J)
//! \param  [out]  Lte            The result projected vector (J^t*diff)
//! \param  [in ]  diff           The error vector
//! \param  [in ]  weight         The weight vector (for robust estimation)
//! \param  [in ]  meters         The points in meters which were used to compute the error
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_jacobian_se3_from_points_weighted_premul_float (
   Rox_Matrix LtL, 
   Rox_Matrix Lte, 
   const Rox_Array2D_Double diff, 
   const Rox_Array2D_Double weight, 
   const Rox_DynVec_Point3D_Float meters
);

//! @} 

#endif
