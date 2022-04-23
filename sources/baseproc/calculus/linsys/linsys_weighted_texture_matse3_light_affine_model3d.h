//==============================================================================
//
//    OPENROX   : File linsys_matse3_light_affine_texture_model3d.h
//
//    Contents  : API of linsys_matse3_light_affine_texture_model3d module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_SE3Z_LIGHT_AFFINE_WEIGHTED_PREMUL__
#define __OPENROX_SE3Z_LIGHT_AFFINE_WEIGHTED_PREMUL__

#include <generated/array2d_float.h>
#include <baseproc/maths/linalg/matrix.h>
#include <baseproc/maths/linalg/matse3.h>
#include <baseproc/maths/linalg/matut3.h>
#include <baseproc/image/imask/imask.h>

//! \ingroup Jacobians
//! \addtogroup se3jacobians
//! @{

//! \brief  Linear system A = L'*L and b = L'*e
//! \brief  Linear System for texture wrt se3, alpha, beta with fixed Z variable
//! \param  [out]  LtL              The 8x8 matrix
//! \param  [out]  Lte              The 8x1 matrix
//! \param  [in ]  K                The camera intrinsic parameters
//! \param  [in ]  T                The pose cTr
//! \param  [in ]  Iu               The image gradient along the u-axis
//! \param  [in ]  Iv               The image gradient along the u-axis
//! \param  [in ]  Z                The depth map
//! \param  [in ]  Ia               The mean of the image (Ic+Ir)/2
//! \param  [in ]  Id               The diff of the image (Ic-Ir)
//! \param  [in ]  weight           The weights
//! \param  [in ]  Im               The image mask
//! \return An error code
//! \todo   To be tested
//! \warning This function suppose that the Z is constant
//! \remark This function should be named 
//! \remark "rox_linsys_weighted_matse3_light_affine_texture_model3d"
ROX_API Rox_ErrorCode rox_linsys_weighted_texture_matse3_light_affine_model3d (
   Rox_Matrix LtL, 
   Rox_Matrix Lte, 
   const Rox_MatUT3 K,
   const Rox_MatSE3 T, 
   const Rox_Array2D_Float Iu, 
   const Rox_Array2D_Float Iv, 
   const Rox_Array2D_Float Z, 
   const Rox_Array2D_Float Ia, 
   const Rox_Array2D_Float Id, 
   const Rox_Array2D_Float weight,
   const Rox_Imask Im
);

//! @} 

#endif
