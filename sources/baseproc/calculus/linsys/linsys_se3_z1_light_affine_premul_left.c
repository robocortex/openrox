//==============================================================================
//
//    OPENROX   : File linsys_se3_z1_light_affine_premul_left.c
//
//    Contents  : Implementation of linsys_se3_z1_light_affine_premul_left module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "linsys_se3_z1_light_affine_premul_left.h"
#include "linsys_se3_z1_light_affine_premul.h"

#include <baseproc/array/multiply/mulmatmat.h>
#include <baseproc/array/multiply/mulmattransmat.h>
#include <baseproc/calculus/linsys/linsys_se3_light_affine_change_update.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_jacobian_se3_z1_light_affine_premul_left (
   Rox_Matrix LtL, 
   Rox_Matrix Lte, 
   const Rox_Array2D_Float Iu, 
   const Rox_Array2D_Float Iv, 
   const Rox_Array2D_Float Ia, 
   const Rox_Array2D_Float Id, 
   const Rox_Imask Im, 
   const Rox_MatSE3 pose, 
   const Rox_MatUT3 K
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !LtL || !Lte ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if ( !pose ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_matse3_check_size ( pose ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Real jacobian with update to the right
   error = rox_linsys_se3_z1_light_affine_premul ( LtL, Lte, Iu, Iv, Ia, Id, Im, pose, K );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Change update to the left
   error = rox_linsys_se3_light_affine_change_update_from_right_to_left ( LtL, Lte, pose );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}
