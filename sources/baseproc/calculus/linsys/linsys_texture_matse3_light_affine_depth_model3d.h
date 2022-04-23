//==============================================================================
//
//    OPENROX   : File linsys_matse3_light_affine_depth_texture_model3d.h
//
//    Contents  : API of linsys_matse3_light_affine_depth_texture_model3d module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __SE3ZGROUPLIGHTAFFINEWEIGHTEDPREMUL__
#define __SE3ZGROUPLIGHTAFFINEWEIGHTEDPREMUL__

#include <generated/array2d_double.h>
#include <generated/array2d_float.h>
#include <generated/array2d_uint.h>
#include <baseproc/maths/linalg/matrix.h>
#include <baseproc/maths/linalg/matse3.h>
#include <baseproc/maths/linalg/matut3.h>
#include <baseproc/image/imask/imask.h>

//! \ingroup Jacobians
//! \addtogroup se3jacobians
//! @{

//! Interaction matrix for texture wrt se3, alpha, beta with depth variable
//! \param  [out]	 LtL           The 8x8 matrix
//! \param  [out]	 Lte           The 8x1 vector
//! \param  [in ]	 Im            The image mask
//! \param  [in ]	 Iu            The image gradient along the u axis
//! \param  [in ]	 Iv            The image gradient along the v axis
//! \param  [in ]	 Z
//! \param  [in ]	 Ia
//! \param  [in ]	 Id
//! \param  [in ]	 Zd            The difference between the reference depth and the warped depth
//! \param  [in ]	 weight
//! \param  [in ]	 T             The pose cTr
//! \param  [in ]	 K             The camera intrinsic parameters
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_jacobian_se3_z_group_light_affine_weighted_premul (
   Rox_Matrix LtL, 
   Rox_Matrix Lte, 
   const Rox_Imask Im, 
   const Rox_Array2D_Float Iu, 
   const Rox_Array2D_Float Iv, 
   const Rox_Array2D_Float Z, 
   const Rox_Array2D_Float Ia, 
   const Rox_Array2D_Float Id, 
   const Rox_Array2D_Float Zd, 
   const Rox_Array2D_Float weight, 
   const Rox_MatSE3 T, 
   const Rox_MatUT3 K
);

//! @} 

#endif
