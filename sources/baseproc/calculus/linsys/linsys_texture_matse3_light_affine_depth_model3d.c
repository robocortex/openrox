//==============================================================================
//
//    OPENROX   : File linsys_texture_matse3_light_affine_depth_model3d.c
//
//    Contents  : Implementation of linsys_texture_matse3_light_affine_depth_model3d module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "linsys_texture_matse3_light_affine_depth_model3d.h"

#include <baseproc/maths/maths_macros.h>
#include <baseproc/array/fill/fillval.h>
#include <baseproc/array/symmetrise/symmetrise.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_jacobian_se3_z_group_light_affine_weighted_premul(
   Rox_Matrix LtL, 
   Rox_Matrix Lte, 
   const Rox_Imask validity, 
   const Rox_Array2D_Float gx_avg, 
   const Rox_Array2D_Float gy_avg, 
   const Rox_Array2D_Float depth, 
   const Rox_Array2D_Float mean, 
   const Rox_Array2D_Float diff, 
   const Rox_Array2D_Float diff_depth, 
   const Rox_Array2D_Float weight, 
   const Rox_MatSE3 pose, 
   const Rox_MatUT3 calib
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Uint ** dvalidity = NULL;
   Rox_Float ** dgx_avg = NULL;
   Rox_Float ** dgy_avg = NULL;
   Rox_Float ** ddepth = NULL;
   Rox_Float ** ddiff = NULL;
   Rox_Float ** ddiffz = NULL;
   Rox_Float ** dmean = NULL;
   Rox_Float ** dweight = NULL;
   Rox_Double ** dk = NULL;

   Rox_Double ** dLtL = NULL;
   Rox_Double  * dLte = NULL;

   // INPUT CHECK
   if (!LtL || !Lte )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if ( !validity || !gx_avg || !gy_avg || !depth || !diff || !diff_depth || !mean || !weight || !pose || !calib) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_matse3_check_size ( pose );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(calib, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(LtL, 8, 8);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(Lte, 8, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint width = 0, height = 0;
   error = rox_array2d_uint_get_size(&height, &width, validity); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_check_size(gx_avg, height, width);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_check_size(gy_avg, height, width);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_check_size(depth, height, width);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_check_size(mean, height, width);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_check_size(diff, height, width);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_check_size(diff_depth, height, width);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_check_size(weight, height, width);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Retrieve pointers

   error = rox_array2d_double_get_data_pointer ( &dLte, Lte );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer_to_pointer( &dLtL, LtL);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_get_data_pointer_to_pointer ( &dvalidity, validity);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_get_data_pointer_to_pointer( &dgx_avg, gx_avg);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_get_data_pointer_to_pointer( &dgy_avg, gy_avg);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_get_data_pointer_to_pointer( &ddepth, depth);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_get_data_pointer_to_pointer( &ddiff, diff);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_get_data_pointer_to_pointer( &ddiffz, diff_depth);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_get_data_pointer_to_pointer( &dmean, mean);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_get_data_pointer_to_pointer( &dweight, weight);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer_to_pointer( &dk, calib);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Retrieve input transformation
   Rox_Double fu = dk[0][0];
   Rox_Double fv = dk[1][1];
   Rox_Double cu = dk[0][2];
   Rox_Double cv = dk[1][2];

   error = rox_array2d_double_fillval(LtL, 0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillval(Lte, 0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Compute per pixel jacobian
   for (Rox_Sint i = 1; i < height - 1; i++)
   {
      Rox_Double J[2][8];

      Rox_Double v = (Rox_Double) i;
      Rox_Double y = (v - cv) / fv;

      for (Rox_Sint j = 1; j < width - 1; j++)
      {
         if (!dvalidity[i][j]) continue;

         // Gradient
         Rox_Double Iu = (Rox_Double)dgx_avg[i][j];
         Rox_Double Iv = (Rox_Double)dgy_avg[i][j];
         Rox_Double w = (Rox_Double)dweight[i][j];

         Rox_Double Z = (Rox_Double)ddepth[i][j];
         Rox_Double u = (Rox_Double)j;
         Rox_Double x = (u - cu) / fu;
         Rox_Double d = w * (Rox_Double)ddiff[i][j];
         Rox_Double dz = w * (Rox_Double)ddiffz[i][j];
         Rox_Double a = (Rox_Double)dmean[i][j];

         Rox_Double X = x * Z;
         Rox_Double Y = y * Z;

         J[0][0] = w * (Iu / Z * fu);
         J[0][1] = w * (Iv / Z * fv);
         J[0][2] = w * ((-Iu * fu * X - Iv * fv * Y) * pow(Z, -0.2e1));
         J[0][3] = w * ((-fv * (Y * Y + Z * Z) * Iv - Iu * Y * fu * X) * pow(Z, -0.2e1));
         J[0][4] = w * ((fu * (X * X + Z * Z) * Iu + Iv * X * fv * Y) * pow(Z, -0.2e1));
         J[0][5] = w * ((Iv * fv * X - Iu * fu * Y) / Z);
         J[0][6] = w * a;
         J[0][7] = w;
         
         J[1][0] = 0;
         J[1][1] = 0;
         J[1][2] = w;
         J[1][3] = w * Y;
         J[1][4] = - w * X;
         J[1][5] = 0;
         J[1][6] = 0;
         J[1][7] = 0;

         // Jt * J
         for (Rox_Sint k = 0; k < 8; k++)
         {
            for (Rox_Sint l = 0; l <= k; l++)
            {
               dLtL[k][l] += J[0][k]*J[0][l] + J[1][k]*J[1][l];
            }

            dLte[k] += J[0][k] * d + J[1][k] * dz;
         }
      }
   }

   // Symmetrise lower triangular input
   error = rox_array2d_double_symmetrise_lower ( LtL );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}
