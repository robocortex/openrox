//==============================================================================
//
//    OPENROX   : File linsys_matsl3_point2d_pix.h
//
//    Contents  : API of linsys_matsl3_point2d_pix module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_SL3POINTS_WEIGHTED_PREMUL__
#define __OPENROX_SL3POINTS_WEIGHTED_PREMUL__

#include <generated/array2d_double.h>
#include <generated/dynvec_point2d_float.h>
#include <baseproc/maths/linalg/matrix.h>

//! \ingroup Jacobians
//! \defgroup sl3jacobians sl3jacobians
//! \brief Jacobians relative to the SL3 group.

//! \addtogroup sl3jacobians
//! @{

//! Compute the jacobian for points wrt sl3
//! 
//! \param  [out]  LtL            The result Hessian Matrix  (J^t*J)
//! \param  [out]  Jtf            The result projected vector (J^t*diff)
//! \param  [in ]  diff           The error vector
//! \param  [in ]  weight         The weight vector (for robust estimation)
//! \param  [in ]  ref            The points in pixels which were used to compute the error
//! \return An error code
//! \todo   To be tested
//! \todo   REname function to rox_linsys_weighted_matsl3_point2d_pix
ROX_API Rox_ErrorCode rox_jacobian_sl3_from_points_weighted_premul_float (
   Rox_Matrix LtL, 
   Rox_Matrix Jtf, 
   const Rox_Array2D_Double diff, 
   const Rox_Array2D_Double weight, 
   const Rox_DynVec_Point2D_Float ref
);

//! @} 

#endif // __OPENROX_SL3POINTS_WEIGHTED_PREMUL__
