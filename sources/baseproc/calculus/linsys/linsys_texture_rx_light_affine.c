//==============================================================================
//
//    OPENROX   : File linsys_texture_rx_light_affine.c
//
//    Contents  : Implementation of linsys_texture_rx_light_affine module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "linsys_texture_rx_light_affine.h"

#include <baseproc/array/fill/fillval.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode linsys_texture_rx_light_affine (
   Rox_Matrix LtL, 
   Rox_Matrix Lte, 
   const Rox_Array2D_Float Id, 
   const Rox_Array2D_Float Iu, 
   const Rox_Imask Im
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !LtL || !Lte )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( !Id || !Iu || !Im )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint width = 0, height = 0;
   error = rox_array2d_float_get_size ( &height, &width, Id ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_check_size ( Im, height, width ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_check_size ( Iu, height, width ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(LtL, 2, 2); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size ( Lte, 2, 1 ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_check_size ( Id, height, width ); 
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

   Rox_Float ** Id_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &Id_data, Id );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** Iu_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &Iu_data, Iu );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint ** Im_data = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer( &Im_data, Im );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint count = 0;
   for (Rox_Sint i = 1; i < height - 1; i++)
   {
      for (Rox_Sint j = 1; j < width - 1; j++)
      {
         Rox_Double L[2];

         if ( !Im_data[i][j] ) continue;

         Rox_Double d = Id_data[i][j];
         Rox_Double Iu_val = Iu_data[i][j];

         L[0] = Iu_val;
         L[1] = 1.0;

         for (Rox_Sint k = 0; k < 2; k++)
         {
            for (Rox_Sint l = 0; l <= k; l++)
            {
               LtL_data[k][l] += L[k]*L[l];
            }

            Lte_data[k][0] += L[k] * d;
         }

         count++;
      }
   }

   for (Rox_Sint k = 0; k < 2; k++)
   {
      for (Rox_Sint l = 0; l <= k; l++)
      {
         LtL_data[l][k] = LtL_data[k][l];
      }
   }

function_terminate:
   return error;
}
