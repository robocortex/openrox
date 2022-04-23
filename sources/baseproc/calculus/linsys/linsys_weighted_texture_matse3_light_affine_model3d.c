//==============================================================================
//
//    OPENROX   : File linsys_weighted_texture_matse3_light_affine_model3d.c
//
//    Contents  : Implementation of linsys weighted texture matse3 light affine model3d module
//
//    Author(s) : R&D department leaded by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "linsys_weighted_texture_matse3_light_affine_model3d.h"

#include <baseproc/array/fill/fillval.h>
#include <baseproc/array/symmetrise/symmetrise.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_linsys_weighted_texture_matse3_light_affine_model3d (
   Rox_Matrix LtL, 
   Rox_Matrix Lte, 
   const Rox_MatUT3 K,
   const Rox_MatSE3 T, 
   const Rox_Array2D_Float Iu, 
   const Rox_Array2D_Float Iv, 
   const Rox_Array2D_Float Z, 
   const Rox_Array2D_Float Ia, 
   const Rox_Array2D_Float Id, 
   const Rox_Array2D_Float weight,
   const Rox_Imask Im
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   // Output check
   if ( !LtL || !Lte )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   // Input check
   if ( !Im || !Iu || !Iv || !Z || !Id || !Ia || !weight || !T || !K ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_matse3_check_size ( T );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matut3_check_size ( K );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size ( LtL, 8, 8 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size ( Lte, 8, 1 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint width = 0, height = 0;
   error = rox_array2d_uint_get_size ( &height, &width, Im ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_check_size ( Iu, height, width );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_check_size ( Iv, height, width );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_check_size ( Z, height, width );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_check_size ( Ia, height, width );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_check_size ( Id, height, width );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_check_size ( weight, height, width );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Retrieve pointers
   Rox_Double  * Lte_data = NULL;
   error = rox_array2d_double_get_data_pointer( &Lte_data, Lte );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** LtL_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &LtL_data, LtL );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint ** Im_data = NULL;
   error = rox_imask_get_data_pointer_to_pointer ( &Im_data, Im );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** Iu_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer( &Iu_data, Iu );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Float ** Iv_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer( &Iv_data, Iv );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Float ** Z_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer( &Z_data, Z );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** Id_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer( &Id_data, Id );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** Ia_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer( &Ia_data, Ia );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** weight_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer( &weight_data, weight );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** K_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &K_data, K );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Double ** T_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &T_data, T );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Retrieve input transformation
   Rox_Double fu = K_data[0][0];
   Rox_Double fv = K_data[1][1];
   Rox_Double cu = K_data[0][2];
   Rox_Double cv = K_data[1][2];

   Rox_Double r11 = T_data[0][0]; Rox_Double r12 = T_data[0][1]; Rox_Double r13 = T_data[0][2];
   Rox_Double r21 = T_data[1][0]; Rox_Double r22 = T_data[1][1]; Rox_Double r23 = T_data[1][2];
   Rox_Double r31 = T_data[2][0]; Rox_Double r32 = T_data[2][1]; Rox_Double r33 = T_data[2][2];
   Rox_Double  tx = T_data[0][3]; Rox_Double  ty = T_data[1][3]; Rox_Double  tz = T_data[2][3];

   // Compute tip for fastening computation: tau = R' * t
   Rox_Double tau1 = ( r11 * tx + r21 * ty + r31 * tz );
   Rox_Double tau2 = ( r12 * tx + r22 * ty + r32 * tz );
   Rox_Double tau3 = ( r13 * tx + r23 * ty + r33 * tz );

   error = rox_array2d_double_fillval ( LtL, 0 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillval ( Lte, 0 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Compute per pixel jacobian (ignore borders)
   // for ( Rox_Sint i = 1; i < height - 1; i++ )
   for ( Rox_Sint i = 0; i < height; i++ )
   {
      Rox_Double v = (Rox_Double) i;
      Rox_Double y = (v - cv) / fv;

      // for ( Rox_Sint j = 1; j < width - 1; j++ )
      for ( Rox_Sint j = 0; j < width; j++ )
      {
         Rox_Double L[8] = { 0.0 };

         if ( Im_data[i][j] == 0 ) 
         {
            continue; 
         }
         
         Rox_Double u = (Rox_Double) j;
         Rox_Double x = (u - cu) / fu;

         // Gradient
         Rox_Double Iu_val = (Rox_Double) Iu_data[i][j];
         Rox_Double Iv_val = (Rox_Double) Iv_data[i][j];
         
         Rox_Double w  = (Rox_Double) weight_data[i][j];

         Rox_Double z = (Rox_Double) Z_data[i][j];
         
         Rox_Double d = w * (Rox_Double) Id_data[i][j];
         Rox_Double a =     (Rox_Double) Ia_data[i][j];

         // TODO: test if Z < eps then continue
         Rox_Double iz = 1.0/z;

         if (z > 500000.0) iz = 0.0;

         Rox_Double t25 = Iu_val * fu;
         Rox_Double t24 = Iv_val * fv;
         Rox_Double t23 = ( x + tau1*iz ) * t25;
         Rox_Double t22 = ( y + tau2*iz ) * t24;
         Rox_Double t21 = 1.0/(tau3*iz+1.0);

         // Interaction matrix row
         L[0] = w * (iz*t25);
         L[1] = w * (iz*t24);
         L[2] = w * (-iz*t21*(t22+t23));
         L[3] = w * ((-((y*tau2+tau3)*iz+y*y+1.0)*t24-y*t23)*t21);
         L[4] = w * ((+((x*tau1+tau3)*iz+x*x+1.0)*t25+x*t22)*t21);
         L[5] = w * (x*t24-y*t25);
         L[6] = w * a;
         L[7] = w;

         // Build the upper triangular part of Lt * L
         for (Rox_Sint k = 0; k < 8; k++)
         {
            for (Rox_Sint l = 0; l <= k; l++)
            {
               LtL_data[k][l] += L[k]*L[l];
            }

            Lte_data[k] += L[k] * d;
         }
      }
   }

   // Symmetrise lower triangular input   
   error = rox_array2d_double_symmetrise_lower ( LtL );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}
