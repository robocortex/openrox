//==============================================================================
//
//    OPENROX   : File linsys_matse3_light_affine_texture_model2d.h
//
//    Contents  : API of linsys_matse3_light_affine_texture_model2d module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_SE3Z_LIGHT_AFFINE_PREMUL__
#define __OPENROX_SE3Z_LIGHT_AFFINE_PREMUL__

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

//! Compute the jacobian for image wrt se3,alpha,beta with depth fixed to 1
//! \param  [out]  LtL            The result Hessian Matrix  (L^t*L)
//! \param  [out]  Lte            The result projected vector (L^t*e)
//! \param  [in ]  Iu             The luminance gradient along u axis
//! \param  [in ]  Iv             The luminance gradient along v axis
//! \param  [in ]  Ia             The reference and warped image average
//! \param  [in ]  Id             The reference and warped image difference
//! \param  [in ]  pose           The camera pose
//! \param  [in ]  K              The camera calibration
//! \param  [in ]  a              The plane parameter
//! \param  [in ]  b              The plane parameter
//! \param  [in ]  c              The plane parameter
//! \param  [in ]  d              The plane parameter
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_linsys_texture_matse3_light_affine_model2d (
   Rox_Matrix LtL, 
   Rox_Matrix Lte, 
   const Rox_MatUT3 K, 
   const Rox_MatSE3 T, 
   const Rox_Double a, 
   const Rox_Double b, 
   const Rox_Double c, 
   const Rox_Double d,
   const Rox_Array2D_Float Iu, 
   const Rox_Array2D_Float Iv, 
   const Rox_Array2D_Float Ia, 
   const Rox_Array2D_Float Id, 
   const Rox_Imask Im
);

//! @} 

#endif
