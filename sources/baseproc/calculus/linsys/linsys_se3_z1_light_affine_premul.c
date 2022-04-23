//==============================================================================
//
//    OPENROX   : File linsys_se3_z1_light_affine_premul.c
//
//    Contents  : Implementation of linsys_se3_z1_light_affine_premul module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "linsys_se3_z1_light_affine_premul.h"
#include "ansi_linsys_se3_z1_light_affine_premul.h"

#include <baseproc/array/fill/fillval.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_linsys_se3_z1_light_affine_premul ( 
   Rox_Matrix LtL, 
   Rox_Matrix Lte, 
   const Rox_Array2D_Float Iu, 
   const Rox_Array2D_Float Iv, 
   const Rox_Array2D_Float Ia, 
   const Rox_Array2D_Float Id, 
   const Rox_Imask Im, 
   const Rox_Array2D_Double pose, 
   const Rox_Array2D_Double K
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !LtL || !Lte ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( !Iu || !Iv || !Ia || !Id || !K || !pose || !Im ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_matse3_check_size ( pose ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matut3_check_size ( K ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size ( LtL, 8, 8 ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size ( Lte, 8, 1 ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_float_get_size ( &rows, &cols, Iu );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_check_size ( Iv, rows, cols ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_check_size ( Id, rows, cols ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_check_size ( Ia, rows, cols ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_imask_check_size ( Im, rows, cols ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double * Lte_data = NULL;
   error = rox_array2d_double_get_data_pointer ( &Lte_data, Lte);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** LtL_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &LtL_data, LtL );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Double ** K_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &K_data, K );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Get pose information as required by jacobian
   Rox_Double ** T_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &T_data, pose );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint ** Im_data = NULL;
   error  = rox_imask_get_data_pointer_to_pointer ( &Im_data, Im );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Float ** Iu_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &Iu_data, Iu );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** Iv_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &Iv_data, Iv );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** Id_data = NULL;
   error  = rox_array2d_float_get_data_pointer_to_pointer ( &Id_data, Id );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** Ia_data = NULL;
   error  = rox_array2d_float_get_data_pointer_to_pointer ( &Ia_data, Ia );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error  = rox_array2d_double_fillval ( LtL, 0 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error  = rox_array2d_double_fillval ( Lte, 0 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_ansi_linsys_se3_z1_light_affine_premul (LtL_data, Lte_data, K_data, T_data, Iu_data, Iv_data, Id_data, Ia_data, Im_data, rows, cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}
