//============================================================================
//
//    OPENROX   : File basegradient.c
//
//    Contents  : Implementation of basegradient module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//============================================================================

#include "basegradient.h"
#include "ansi_basegradient.h"

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_array2d_float_basegradient (
   Rox_Array2D_Float Iu, 
   Rox_Array2D_Float Iv, 
   Rox_Imask Gm, 
   const Rox_Array2D_Float I, 
   const Rox_Imask Im
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   // Check inputs
   if ( !I || !Im ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   // Check outputs
   if ( !Iu || !Iv || !Gm )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_float_get_size ( &rows, &cols, I );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_check_size ( Iu, rows, cols ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_check_size ( Iv, rows, cols ); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_imask_check_size ( Gm, rows, cols ); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_imask_check_size ( Im, rows, cols ); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   // Get data of input mask
   Rox_Uint ** Im_data = NULL;
   error = rox_imask_get_data_pointer_to_pointer ( &Im_data, Im );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Get data of output mask
   Rox_Uint ** Gm_data = NULL;
   error = rox_imask_get_data_pointer_to_pointer ( &Gm_data, Gm );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** I_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &I_data, I );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Float ** Iu_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &Iu_data, Iu );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Float ** Iv_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &Iv_data, Iv );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_ansi_array2d_float_basegradient ( Iu_data, Iv_data, Gm_data, I_data, Im_data, rows, cols );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_float_basegradient_nomask (
   Rox_Array2D_Float Iu, 
   Rox_Array2D_Float Iv, 
   const Rox_Array2D_Float I
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   // Check inputs
   if ( !I ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   // Check outputs
   if ( !Iu || !Iv )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_float_get_size ( &rows, &cols, I );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_check_size ( Iu, rows, cols ); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_float_check_size ( Iv, rows, cols ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** I_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &I_data, I );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** Iu_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &Iu_data, Iu );
   ROX_ERROR_CHECK_TERMINATE ( error );
  
   Rox_Float ** Iv_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &Iv_data, Iv );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_ansi_array2d_float_basegradient_nomask ( Iu_data, Iv_data, I_data, rows, cols );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}
