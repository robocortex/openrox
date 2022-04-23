//==============================================================================
//
//    OPENROX   : File linsys_se3_light_affine_premul_left.c
//
//    Contents  : Implementation of linsys_se3z1_light_affine_premul_left module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "linsys_se3_light_affine_premul_left.h"
#include "linsys_se3_light_affine_premul.h"

#include <baseproc/array/multiply/mulmatmat.h>
#include <baseproc/array/multiply/mulmattransmat.h>
#include <baseproc/calculus/linsys/linsys_se3_light_affine_change_update.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_jacobian_se3_light_affine_premul_left (
   Rox_Matrix LtL, 
   Rox_Matrix Lte, 
   const Rox_Array2D_Float gx, 
   const Rox_Array2D_Float gy, 
   const Rox_Array2D_Float mean, 
   const Rox_Array2D_Float diff, 
   const Rox_Imask mask, 
   const Rox_MatSE3 pose, 
   const Rox_MatUT3 calib_input
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!LtL || !Lte) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (!pose) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_matse3_check_size ( pose ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Real linear system with update to the right
   error = rox_jacobian_se3_light_affine_premul ( LtL, Lte, gx, gy, mean, diff, mask, pose, calib_input );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_linsys_se3_light_affine_change_update_from_right_to_left ( LtL, Lte, pose );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}
