//==============================================================================
//
//    OPENROX   : File linsys_texture_matse3_light_affine_model3d_zi.c
//
//    Contents  : Implementation of linsys_texture_matse3_light_affine_model3d_zi module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "linsys_texture_matse3_light_affine_model3d_zi.h"

#include <float.h>
#include <math.h>

#include "ansi_linsys_texture_matse3_light_affine_model3d_zi.h"

#include <baseproc/maths/linalg/matrix.h>
#include <baseproc/array/fill/fillzero.h>
#include <baseproc/array/symmetrise/symmetrise.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_linsys_texture_matse3_light_affine_model3d_zi (
   Rox_Matrix LtL, 
   Rox_Matrix Lte,
   const Rox_MatUT3 K,
   const Rox_MatSE3 T,
   const Rox_Array2D_Float Zi, 
   const Rox_Array2D_Float Ziu, 
   const Rox_Array2D_Float Ziv, 
   const Rox_Array2D_Float Iu, 
   const Rox_Array2D_Float Iv, 
   const Rox_Array2D_Float Id, 
   const Rox_Array2D_Float Ia, 
   const Rox_Imask Im
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   // Output check
   if ( !LtL || !Lte )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   // Input check
   if ( !Im || !Iu || !Iv || !Zi || !Ziu || !Ziv || !Id || !Ia || !T || !K ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_float_get_size ( &rows, &cols, Zi );
   ROX_ERROR_CHECK_TERMINATE ( error );
      // Check input sizes
   error = rox_matse3_check_size ( T ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matut3_check_size ( K ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size ( LtL, 8, 8 ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size ( Lte, 8, 1 ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_check_size ( Iv, rows, cols ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_check_size ( Iu, rows, cols ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_check_size ( Id, rows, cols ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_check_size ( Ia, rows, cols ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_check_size ( Zi, rows, cols ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_check_size ( Ziu, rows, cols ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_check_size ( Ziv, rows, cols ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_check_size ( Im, rows, cols ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Get pointers to data
   Rox_Double ** LtL_data =  NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &LtL_data, LtL );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillzero ( LtL );
   ROX_ERROR_CHECK_TERMINATE ( error  );

   Rox_Double ** Lte_data =  NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &Lte_data, Lte );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillzero ( Lte );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint ** Im_data = NULL;
   error = rox_imask_get_data_pointer_to_pointer ( &Im_data, Im );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** Id_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &Id_data, Id );
   ROX_ERROR_CHECK_TERMINATE (error );

   Rox_Float ** Ia_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer( &Ia_data, Ia );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** Iu_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &Iu_data, Iu );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** Iv_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &Iv_data, Iv );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** Zi_data  =  NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &Zi_data, Zi );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** Ziu_data =  NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &Ziu_data, Ziv );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** Ziv_data =  NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &Ziv_data, Ziu );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_MatSE3 Ti = NULL;
   error = rox_matse3_new ( &Ti );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_matse3_inv ( Ti, T );
   ROX_ERROR_CHECK_TERMINATE ( error );

   //rox_matse3_print(T);
   //rox_matse3_print(Ti);

   // NB tau = -inv(cRr)*ctr = rtc 
   Rox_Array2D_Double tau = NULL;
   error = rox_array2d_double_new_subarray2d ( &tau, Ti, 0, 3, 3, 1 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** tau_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &tau_data, tau );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** K_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &K_data, K );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Compute Lte and lower triangular part of LtL
   error = rox_ansi_linsys_texture_matse3_light_affine_model3d_zi ( LtL_data, Lte_data, K_data, tau_data, Zi_data, Ziu_data, Ziv_data, Iu_data, Iv_data, Id_data, Ia_data, Im_data, rows, cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Symmetrise by copying the lower triangular part into the upper triangular part 
   error = rox_array2d_double_symmetrise_lower ( LtL );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   rox_matse3_del ( &Ti );
   rox_array2d_double_del ( &tau );
   return error;
}
