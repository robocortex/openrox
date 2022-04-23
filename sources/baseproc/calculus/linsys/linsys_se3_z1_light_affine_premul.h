//==============================================================================
//
//    OPENROX   : File linsys_se3_z1_light_affine_premul.h
//
//    Contents  : API of linsys_se3_z1_light_affine_premul module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_LINSYS_SE3Z1_LIGHT_AFFINE_PREMUL__
#define __OPENROX_LINSYS_SE3Z1_LIGHT_AFFINE_PREMUL__

#include <generated/array2d_double.h>
#include <generated/array2d_float.h>
#include <baseproc/maths/linalg/matse3.h>
#include <baseproc/maths/linalg/matut3.h>
#include <baseproc/maths/linalg/matrix.h>
#include <baseproc/image/imask/imask.h>

//! \ingroup Jacobians
//! \addtogroup se3jacobians
//! @{

//! Compute the linear system for image relative to matse3, alpha, beta with depth shifted by -1
//! We suppose that the template is an image of the 3D plane that was captured with a camera
//! with parameters K_r = "calib_input" and with a pose r_T_o = [eye(3); [0;0;1]; 0,0,0,1]
//! and that the current pose c_T_o has been shifted: c_T_o = c_T_o * [eye(3); [0;0;-1]; 0,0,0,1]
//! The jacobian is in rox_mat file jacobian_matse3_model_2d_image.m
//! H = Kr * inv(eye(3)-otc*transpose(no)) * (R + (t-otc)*transpose(no)) * inv(Kr)
//! \param  [out]  LtL            The result Hessian Matrix Approximation (J^t*J) of size (8x8)    
//! \param  [out]  Lte            The result projected vector (J^t*diff) of size (8x1) 
//! \param  [in ]  gx             The luminance gradient in x vector
//! \param  [in ]  gy             The luminance gradient in y vector
//! \param  [in ]  mean           The Reference plus current image mean
//! \param  [in ]  diff           The error image
//! \param  [in ]  mask           The valid pixel image
//! \param  [in ]  pose           The camera pose cTo
//! \param  [in ]  calib_input    The camera calibration 
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_linsys_se3_z1_light_affine_premul (
   Rox_Matrix LtL, 
   Rox_Matrix Lte, 
   const Rox_Array2D_Float gx, 
   const Rox_Array2D_Float gy, 
   const Rox_Array2D_Float mean, 
   const Rox_Array2D_Float diff, 
   const Rox_Imask mask, 
   const Rox_MatSE3 pose, 
   const Rox_MatUT3 calib_input
);

//! @}

#endif // __OPENROX_LINSYS_SE3Z1_LIGHT_AFFINE_PREMUL__
