//============================================================================
//
//    OPENROX   : File gradient_anglenorm.h
//
//    Contents  : API of gradient_anglenorm module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//============================================================================

#ifndef __OPENROX_GRADIENT_ANGLENORM__
#define __OPENROX_GRADIENT_ANGLENORM__

#include <generated/array2d_uint.h>
#include <generated/array2d_sint.h>
#include <generated/array2d_float.h>
#include <generated/array2d_point2d_sshort.h>
#include <baseproc/geometry/point/point2d.h>
#include <baseproc/image/image.h>

//! \ingroup Vision
//! \addtogroup Gradient
//! @{

//! Compute gradient angle = atan2(gv, gu) and norm = sqrt(gu^2+gv^2) from precomputed gradients 
//! If the gradient norm is below the threshold, norm and angle are set to 0
//! \param  [out]  angle                  The gradient angle (i.e. the angle theta of the normal vector n = [cost(theta); sin(theta)])
//! \param  [out]  norm                   The norm of the gradient
//! \param  [in ]  gu                     The gradient along the u-axis
//! \param  [in ]  gv                     The gradient along the v-axis
//! \param  [in ]  norm_threshold         The threshold of the norm below which the angle and norm are set to 0
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_array2d_float_gradient_angle_norm_nomask (
   Rox_Array2D_Float angle, 
   Rox_Array2D_Float norm, 
   const Rox_Array2D_Float gu, 
   const Rox_Array2D_Float gv,
   const Rox_Float norm_threshold
);

//! Compute gradient angle = atan2(gv, gu) and scale = gu^2+gv^2 from precomputed gradients
//! If the gradient norm is below the threshold, norm and angle are set to 0
//! \param  [out]  angle                  The gradient angle (i.e. the angle theta of the normal vector n = [cost(theta); sin(theta)])
//! \param  [out]  scale                  The norm of the gradient
//! \param  [in ]  gu                     The gradient along the u-axis
//! \param  [in ]  gv                     The gradient along the v-axis
//! \param  [in ]  scale_threshold        The threshold of the norm below which the angle and norm are set to 0
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_array2d_float_gradient_angle_scale_nomask (
   Rox_Array2D_Float angle, 
   Rox_Array2D_Float scale, 
   Rox_Array2D_Float gu, 
   Rox_Array2D_Float gv,
   Rox_Float norm_threshold
);

//! Compute gradient angle = atan2(gv, gu) and scale = gu^2+gv^2 from precomputed gradients
//! If the gradient norm is below the threshold, norm and angle are set to 0
//! \param  [out]  angle                  The gradient angle (i.e. the angle theta of the normal vector n = [cost(theta); sin(theta)])
//! \param  [out]  scale                  The norm of the gradient
//! \param  [in ]  gu                     The gradient along the u-axis
//! \param  [in ]  gv                     The gradient along the v-axis
//! \param  [in ]  scale_threshold        The threshold of the norm below which the angle and norm are set to 0
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_array2d_sint_gradient_angle_scale_nomask (
   Rox_Array2D_Float angle, 
   Rox_Array2D_Uint scale, 
   Rox_Array2D_Sint gu, 
   Rox_Array2D_Sint gv,
   Rox_Uint norm_threshold
);

//! Compute the scale = gu^2+gv^2 and angle = atan2(gu, gv) of an image gradient
//! \param  [out]  angle             The result angle of gradient (i.e. the angle theta of the normal vector n = [cost(theta); sin(theta)])
//! \param  [out]  scale             The result scale of gradient
//! \param  [int]  image_gradient    The gradient to consider
//! \return An error code
ROX_API Rox_ErrorCode rox_point2d_sshort_angle_scale (
   Rox_Double * scale, 
   Rox_Double * angle, 
   const Rox_Point2D_Sshort image_gradient
);


//! Compute the norm = gu^2+gv^2 and angle = atan2(gu, gv) of an image gradient
//! \param  [out]  angle             The result angle of gradient (i.e. the angle theta of the normal vector n = [cost(theta); sin(theta)])
//! \param  [out]  norm              The result norm of gradient
//! \param  [int]  image_gradient    The gradient to consider
//! \return An error code
ROX_API Rox_ErrorCode rox_point2d_sshort_angle_scale (
   Rox_Double * norm, 
   Rox_Double * angle, 
   const Rox_Point2D_Sshort image_gradient
);


//! Compute gradient angle = atan2(gu, gv) and norm = sqrt(gu^2+gv^2) from precomputed gradients
//! \param  [out]  angle          The gradient angle (i.e. the angle theta of the normal vector n = [cost(theta); sin(theta)])
//! \param  [out]  norm           The norm of the gradient
//! \param  [in ]  gu             The gradient along the u-axis
//! \param  [in ]  gv             The gradient along the v-axis
//! \return An error code
//! \todo   To be tested
// ROX_API Rox_ErrorCode rox_image_gradient_angle_scale_nomask (
//    Rox_Array2D_Float angle, 
//    Rox_Array2D_Sint  scale, 
//    Rox_Array2D_Sint gu, 
//    Rox_Array2D_Sint gv,
//    Rox_Sint scale_threshold
// );


//! Compute gradient angle = atan2(gu, gv) and scale = (gu^2+gv^2) from image
//! \param  [out]  angle          The gradient angle (i.e. the angle theta of the normal vector n = [cost(theta); sin(theta)])
//! \param  [out]  norm           The norm of the gradient
//! \param  [in ]  gu             The gradient along the u-axis
//! \param  [in ]  gv             The gradient along the v-axis
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_image_gradient_sobel_angle_scale_nomask (
   Rox_Array2D_Float angle, 
   Rox_Array2D_Uint  scale, 
   Rox_Image image_gray,
   Rox_Sint scale_threshold
);

ROX_API Rox_ErrorCode rox_image_gradient_sobel_angle_scale_nomask_buffers (
   Rox_Float * angle, 
   Rox_Uint * scale, 
   Rox_Image image_gray,
   Rox_Uint scale_threshold
);

//! @}

#endif
