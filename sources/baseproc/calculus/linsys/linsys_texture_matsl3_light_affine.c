//==============================================================================
//
//    OPENROX   : File linsys_texture_matsl3_light_affine.c
//
//    Contents  : Implementation of linsys_texture_matsl3_light_affine module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "linsys_texture_matsl3_light_affine.h"
#include "ansi_linsys_texture_matsl3_light_affine.h"

#include <baseproc/array/fill/fillval.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode linsys_texture_matsl3_light_affine (
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

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_float_get_size(&rows, &cols, Id); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_check_size(Im, rows, cols); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_check_size(Iu, rows, cols); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_check_size(Iv, rows, cols); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_check_size(Ia, rows, cols); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_check_size(Id, rows, cols); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(LtL, 10, 10); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(Lte, 10, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillval(LtL, 0.0); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillval(Lte, 0.0); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double **LtL_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &LtL_data, LtL);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** Lte_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &Lte_data, Lte);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Float ** Id_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer( &Id_data, Id);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Float ** Ia_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer( &Ia_data, Ia);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** Iu_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer( &Iu_data, Iu);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** Iv_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer( &Iv_data, Iv);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint ** Im_data = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer( &Im_data, Im);
   ROX_ERROR_CHECK_TERMINATE ( error );

   if(1)
   {

   error = rox_ansi_linsys_texture_matsl3_light_affine ( LtL_data, Lte_data[0], Iu_data, Iv_data, Id_data, Ia_data, Im_data, rows, cols );
   ROX_ERROR_CHECK_TERMINATE ( error );

   }
else
{
   for (Rox_Sint i = 0; i < rows; i++)
   {
      Rox_Double v = (Rox_Double) i;

      for (Rox_Sint j = 0; j < cols; j++)
      {
         Rox_Double J[10];

         if (!Im_data[i][j]) continue;

         Rox_Double u = (Rox_Double) j;

         Rox_Double d = Id_data[i][j];
         Rox_Double Iu = Iu_data[i][j];
         Rox_Double Iv = Iv_data[i][j];
         Rox_Double z = -Iv * v;
         Rox_Double w =  Iu * u;
         Rox_Double temp = z - w;

         J[0] = Iu;
         J[1] = Iv;
         J[2] = Iu * v;
         J[3] = Iv * u;
         J[4] = w + z;
         J[5] = temp + z;
         J[6] = temp * u;
         J[7] = temp * v;
         J[8] = Ia_data[i][j];
         J[9] = 1.0;

         for (Rox_Sint k = 0; k < 10; k++)
         {
            for (Rox_Sint l = 0; l <= k; l++)
            {
               LtL_data[k][l] += J[k]*J[l];
            }

            Lte_data[k][0] += J[k] * d;
         }
      }
   }

   for (Rox_Sint k = 0; k < 10; k++)
   {
      for (Rox_Sint l = 0; l <= k; l++)
      {
         LtL_data[l][k] = LtL_data[k][l];
      }
   }
}

function_terminate:
   return error;
}
