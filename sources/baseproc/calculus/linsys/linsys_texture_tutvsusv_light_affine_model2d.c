//==============================================================================
//
//    OPENROX   : File linsys_texture_tutvsusv_light_affine_model2d.c
//
//    Contents  : Implementation of linsys_texture_tutvsusv_light_affine_model2d module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "linsys_texture_tutvsusv_light_affine_model2d.h"

#include <baseproc/array/fill/fillval.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_jacobian_tutvsusv_light_affine_premul (
   Rox_Matrix LtL, 
   Rox_Matrix Lte, 
   const Rox_Array2D_Float Ia, 
   const Rox_Array2D_Float Id, 
   const Rox_Array2D_Float Iu, 
   const Rox_Array2D_Float Iv, 
   const Rox_Imask Im
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!LtL || !Lte )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( !Id || !Iu || !Iv || !Ia || !Im) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint width = 0, height = 0;
   error = rox_array2d_float_get_size ( &height, &width, Id ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_check_size ( Im, height, width ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_check_size ( Iu, height, width ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_check_size ( Iv, height, width ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_check_size ( Ia, height, width ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_check_size ( Id, height, width ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size ( LtL, 6, 6 ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size ( Lte, 6, 1 ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillval ( LtL, 0.0 ); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_fillval ( Lte, 0.0 ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** LtL_data = NULL; 
   error = rox_array2d_double_get_data_pointer_to_pointer ( &LtL_data, LtL );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Double ** Lte_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &Lte_data, Lte );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** Id_data    = NULL; 
   error = rox_array2d_float_get_data_pointer_to_pointer ( &Id_data, Id );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** Ia_data    = NULL; 
   error = rox_array2d_float_get_data_pointer_to_pointer ( &Ia_data, Ia );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** Iu_data   = NULL; 
   error = rox_array2d_float_get_data_pointer_to_pointer ( &Iu_data, Iu );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** Iv_data   = NULL; 
   error = rox_array2d_float_get_data_pointer_to_pointer ( &Iv_data, Iv );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint ** Im_data     = NULL; 
   error = rox_array2d_uint_get_data_pointer_to_pointer ( &Im_data, Im );
   ROX_ERROR_CHECK_TERMINATE ( error );

   for ( Rox_Sint i = 0; i < height; i++)
   {
      Rox_Double v = (Rox_Double)i;

      for ( Rox_Sint j = 0; j < width; j++)
      {
         Rox_Double L[6];

         if ( !Im_data[i][j] ) continue;

         Rox_Double u = (Rox_Double) j;

         Rox_Double d  = Id_data[i][j];
         Rox_Double Iu_val = Iu_data[i][j];
         Rox_Double Iv_val = Iv_data[i][j];

         Rox_Double z = -Iv_val * v;
         Rox_Double w =  Iu_val * u;

         L[0] = Iu_val;
         L[1] = Iv_val;
         L[2] = z + w;
         L[3] = 2.0 * z - w;
         L[4] = Ia_data[i][j];
         L[5] = 1.0;

         for ( Rox_Sint k = 0; k < 6; k++)
         {
            for ( Rox_Sint l = 0; l <= k; l++)
            {
               LtL_data[k][l] += L[k]*L[l];
            }

            Lte_data[k][0] += L[k] * d;
         }
      }
   }

   for ( Rox_Sint k = 0; k < 6; k++)
   {
      for ( Rox_Sint l = 0; l <= k; l++)
      {
         LtL_data[l][k] = LtL_data[k][l];
      }
   }

function_terminate:
   return error;
}
