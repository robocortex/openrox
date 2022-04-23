//==============================================================================
//
//    OPENROX   : File linsys_point2d_nor_matse3_matso3z_matso3z.h
//
//    Contents  : API of linsys_point2d_nor_matse3_matso3z_matso3z
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_SE3_SO3Z_SO3Z_INTMAT__
#define __OPENROX_SE3_SO3Z_SO3Z_INTMAT__

#include <generated/array2d_double.h>
#include <generated/dynvec_point2d_float.h>
#include <generated/dynvec_point3d_float.h>
#include <baseproc/maths/linalg/matrix.h>

//! \ingroup Jacobians
//! \defgroup se3jacobians se3jacobians
//! \brief Jacobians relative to the SE3 group.

//! \addtogroup se3jacobians
//!  @{

//! Compute the interaction matrix for two sets of points
//! related by a transformation with an unknown in XY-plane rotation
//! prepare its pseudo-inversion
//! \param  [out]  LtL           the result L^t*L Matrix
//! \param  [out]  Lte           the result projected vector (L^t*diff)
//! \param  [in ]  diff_b        the error vector for the b points
//! \param  [in ]  weight_b      the weight vector for the b points (for robust estimation)
//! \param  [in ]  mb_c          the b points, in meters, used to compute the error,_replaced_in_the_camera_frame_
//! \param  [in ]  diff_s        the error vector for the s points
//! \param  [in ]  weight_s      the weight vector for the s points (for robust estimation)
//! \param  [in ]  ms_c          the s points, in meters, used to compute the error,_replaced_in_the_camera_frame_
//! \param  [in ]  msu_g         utility vectors needed to update the bTg rotation
//! \param  [in ]  msu_s         utility vectors needed to update the pTs rotation
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_linsys_point2d_nor_matse3_matso3z_matso3z (
   Rox_Matrix       LtL,
   Rox_Matrix       Lte,
   const Rox_Array2D_Double       diff_b,
   const Rox_Array2D_Double       weight_b,
   const Rox_DynVec_Point3D_Float mb_c,
   const Rox_Array2D_Double       diff_s,
   const Rox_Array2D_Double       weight_s,
   const Rox_DynVec_Point3D_Float ms_c,
   const Rox_DynVec_Point3D_Float msu_g, 
   const Rox_DynVec_Point3D_Float msu_s 
);

//! @}

#endif
