//==============================================================================
//
//    OPENROX   : File linsys_weighted_texture_matse3_light_affine_model3d_zi.c
//
//    Contents  : Implementation of linsys_weighted_texture_matse3_light_affine_model3d_zi module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "linsys_weighted_texture_matse3_light_affine_model3d_zi.h"

#include <float.h>
#include <math.h>

#include <baseproc/calculus/jacobians/interaction_row_texture_matse3_model3d_zi.h>
#include <baseproc/maths/linalg/matrix.h>
#include <baseproc/array/fill/fillzero.h>
#include <baseproc/array/symmetrise/symmetrise.h>
#include <inout/system/print.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_linsys_weighted_texture_matse3_light_affine_model3d_zi (
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
   const Rox_Array2D_Float weight,
   const Rox_Imask Im
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Sint u_ini = 0;
   Rox_Sint v_ini = 0;

   // Output check
   if ( !LtL || !Lte )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   // Input check
   if ( !Im || !Iu || !Iv || !Zi || !Ziu || !Ziv || !Id || !T || !K ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_fillzero ( LtL );
   ROX_ERROR_CHECK_TERMINATE ( error  );

   error = rox_array2d_double_fillzero ( Lte );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_float_get_size ( &rows, &cols, Zi );
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

   Rox_Float ** weight_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer( &weight_data, weight );
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

   Rox_Double ** LtL_data =  NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &LtL_data, LtL );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** Lte_data =  NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &Lte_data, Lte );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_MatSE3 Ti = NULL;
   error = rox_matse3_new ( &Ti );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_matse3_inv ( Ti, T );
   ROX_ERROR_CHECK_TERMINATE ( error );

   //rox_matse3_print(T);
   //rox_matse3_print(Ti);

   Rox_Array2D_Double tau = NULL;
   error = rox_array2d_double_new_subarray2d ( &tau, Ti, 0, 3, 3, 1 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint v = 0; v < rows; v++)
   {
      Rox_Double vr = (Rox_Double) (v + v_ini);

      for (Rox_Sint u = 0; u < cols; u++ )
      {
         Rox_Double L_row[8] = { 0.0 };

         if ( Im_data[v][u] == 0 ) 
         {
            continue; 
         }
         
         Rox_Double ur = (Rox_Double) (u + u_ini);

         Rox_Double Iu_value = (Rox_Double) Iu_data[v][u];
         Rox_Double Iv_value = (Rox_Double) Iv_data[v][u];

         Rox_Double w  = (Rox_Double) weight_data[v][u];

         Rox_Double zi  = Zi_data [v][u];
         Rox_Double ziu = Ziu_data[v][u];
         Rox_Double ziv = Ziv_data[v][u];
         
         Rox_Double e = (Rox_Double) Id_data[v][u];
         Rox_Double a = (Rox_Double) Ia_data[v][u];

         error = rox_interaction_row_texture_matse3_model3d_zi ( L_row, ur, vr, Iu_value, Iv_value, zi, ziu, ziv, K, tau );
         ROX_ERROR_CHECK_TERMINATE ( error );
         
         // Interaction matrix for the light affine model
         L_row[6] =   a;
         L_row[7] = 1.0;
         
         // Update lower triangular part of the system
         for (Rox_Sint k = 0; k < 8; k++)
         {
            for (Rox_Sint l = 0; l <= k; l++)
            {
               LtL_data[k][l] += L_row[k] * L_row[l] * w * w ;
            }

            Lte_data[k][0] += L_row[k] * e * w * w;
         }

      }
   }

   // Symmetrise lower triangular input
   error = rox_array2d_double_symmetrise_lower ( LtL );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   rox_matse3_del ( &Ti );
   rox_array2d_double_del ( &tau );
   return error;
}
