//==============================================================================
//
//    OPENROX   : File linsys_texture_matse2_light_affine_model2d.c
//
//    Contents  : Implementation of linsys_texture_matse2_light_affine_model2d module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "linsys_texture_matse2_light_affine_model2d.h"

#include <baseproc/array/fill/fillval.h>
#include <baseproc/array/symmetrise/symmetrise.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_jacobian_se2_light_affine_premul (
   Rox_Matrix LtL, 
   Rox_Matrix Lte, 
   const Rox_Array2D_Float gx, 
   const Rox_Array2D_Float gy, 
   const Rox_Array2D_Float mean, 
   const Rox_Array2D_Float diff, 
   const Rox_Imask mask, 
   const Rox_Array2D_Double pose_se2, 
   const Rox_Array2D_Double calib_input
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint cols, rows;
   Rox_Double **dki, **dT, **dLtL, *dLte;
   Rox_Float **dgx, ** dgy, ** dd, **da;
   Rox_Float J[6];
   Rox_Uint **dm;
   Rox_Float fu, fv, cu, cv;
   Rox_Float ur, vr, Iu, Iv, d;
   Rox_Float r1,r2,r3,r4,a;

   if (!LtL || !Lte) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (!gx || !gy || !mean || !diff || !calib_input || !pose_se2 || !mask) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_check_size(pose_se2, 3, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(calib_input, 3, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(LtL, 5, 5); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(Lte, 5, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_get_size(&rows, &cols, gx); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_check_size(gy, rows, cols); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_check_size(gx, rows, cols); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_check_size(diff, rows, cols); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_check_size(mean, rows, cols); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_uint_check_size(mask, rows, cols); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer ( &dLte, Lte);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer_to_pointer( &dLtL, LtL);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer_to_pointer( &dki, calib_input);
   ROX_ERROR_CHECK_TERMINATE ( error );

   fu = (Rox_Float) dki[0][0];
   fv = (Rox_Float) dki[1][1];
   cu = (Rox_Float) dki[0][2];
   cv = (Rox_Float) dki[1][2];

   // Get pose information as required by jacobian
   error = rox_array2d_double_get_data_pointer_to_pointer( &dT, pose_se2);
   r1 = (Rox_Float) dT[0][0];
   r2 = (Rox_Float) dT[0][1];
   r3 = (Rox_Float) dT[1][0];
   r4 = (Rox_Float) dT[1][1];

   error = rox_array2d_uint_get_data_pointer_to_pointer( &dm, mask);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_float_get_data_pointer_to_pointer( &dgx, gx);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_get_data_pointer_to_pointer( &dgy, gy);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_get_data_pointer_to_pointer( &dd, diff);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_get_data_pointer_to_pointer( &da, mean);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillval(LtL, 0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillval(Lte, 0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for ( Rox_Sint i = 0; i < rows; i++)
   {
      vr = (float)(i);

      for ( Rox_Sint j = 0; j < cols; j++)
      {
         if (!dm[i][j]) continue;

         ur = (float)(j);

         // Retrieve per pixel params
         Iu = dgx[i][j];
         Iv = dgy[i][j];
         d = dd[i][j];
         a = da[i][j];

         // Jacobian row
         J[0] = Iu * fu * r1 + Iv * fv * r3;
         J[1] = Iu * fu * r2 + Iv * fv * r4;
         J[2] = (-Iu * r1 * (vr - cv) * fu * fu + (r2 * (ur - cu) * Iu - r3 * Iv * (vr - cv)) * fv * fu + Iv * fv * fv * r4 * (ur - cu)) / fu / fv;

         // Update system
         for ( Rox_Sint k = 0; k < 3; k++)
         {
            for ( Rox_Sint l = 0; l <= k; l++)
            {
               dLtL[k][l] += J[k] * J[l];
            }

            dLte[k] += J[k] * d;
         }

         dLtL[3][0] += J[0] * a;
         dLtL[3][1] += J[1] * a;
         dLtL[3][2] += J[2] * a;
         dLtL[3][3] += a * a;

         dLtL[4][0] += J[0];
         dLtL[4][1] += J[1];
         dLtL[4][2] += J[2];
         dLtL[4][3] += a;
         dLtL[4][4] += 1;

         dLte[3] += a * d;
         dLte[4] += d;
      }
   }

   error = rox_array2d_double_symmetrise_lower ( LtL );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}
