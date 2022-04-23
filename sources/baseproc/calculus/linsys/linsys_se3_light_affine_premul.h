//==============================================================================
//
//    OPENROX   : File linsys_se3_light_affine_premul.h
//
//    Contents  : API of linsys_se3_light_affine_premul module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_LINSYS_SE3_LIGHT_AFFINE_PREMUL__
#define __OPENROX_LINSYS_SE3_LIGHT_AFFINE_PREMUL__

#include <generated/array2d_double.h>
#include <generated/array2d_float.h>

#include <baseproc/maths/linalg/matse3.h>
#include <baseproc/maths/linalg/matut3.h>
#include <baseproc/maths/linalg/matrix.h>
#include <baseproc/image/imask/imask.h>

//! \ingroup Jacobians
//! \addtogroup se3jacobians
//! @{

//! Compute the jacobian for image relative to matse3, alpha, beta 
//! We suppose that the template is an image of the 3D plane that was captured with a camera
//! with parameters K_r = "K" and with a pose r_T_o = [eye(3); [0;0;1]; 0,0,0,1]
//! The jacobian is in rox_mat file jacobian_matse3_model_2d_image.m
//! H = Kr * inv(Qo-(otc+no)*transpose(no)) * (R*Qo + (t-(otc+no))*transpose(no)) * inv(Kr)
//! \param  [out]  LtL            The result Hessian Matrix Approximation (J^t*J) of size (8x8)    
//! \param  [out]  Lte            The result projected vector (J^t*Id) of size (8x1) 
//! \param  [in ]  Iu             The luminance gradient in x vector
//! \param  [in ]  Iv             The luminance gradient in y vector
//! \param  [in ]  Ia             The reference plus current image Ia
//! \param  [in ]  Id             The error image
//! \param  [in ]  Im             The valid pixel image
//! \param  [in ]  T              The camera pose cTo
//! \param  [in ]  K              The camera calibration 
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_jacobian_se3_light_affine_premul ( 
   Rox_Matrix LtL, 
   Rox_Matrix Lte, 
   const Rox_Array2D_Float Iu, 
   const Rox_Array2D_Float Iv, 
   const Rox_Array2D_Float Ia, 
   const Rox_Array2D_Float Id, 
   const Rox_Imask Im, 
   const Rox_MatSE3 T, 
   const Rox_MatUT3 K
);

//! @}

#endif // __OPENROX_SE3_LIGHT_AFFINE_PREMUL__
