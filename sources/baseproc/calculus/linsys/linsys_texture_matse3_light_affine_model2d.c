//==============================================================================
//
//    OPENROX   : File linsys_texture_matse3_light_affine_model2d.c
//
//    Contents  : Implementation of linsys_texture_matse3_light_affine_model2d module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "linsys_texture_matse3_light_affine_model2d.h"

#include <float.h>

#include <baseproc/maths/maths_macros.h>
#include <baseproc/array/fill/fillval.h>
#include <baseproc/array/symmetrise/symmetrise.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_linsys_texture_matse3_light_affine_model2d (
   Rox_Matrix LtL,
   Rox_Matrix Lte,
   const Rox_MatUT3 K,
   const Rox_MatSE3 T,
   const Rox_Double a,
   const Rox_Double b,
   const Rox_Double c,
   const Rox_Double d,
   const Rox_Array2D_Float Iu,
   const Rox_Array2D_Float Iv,
   const Rox_Array2D_Float Ia,
   const Rox_Array2D_Float Id,
   const Rox_Imask Im
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   // Check input
   if ( !LtL || !Lte )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( !Iu || !Iv || !Ia || !Id || !K || !T || !Im )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (fabs(d) < DBL_EPSILON)
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Check sizes
   Rox_Sint width = 0, height = 0;
   error = rox_array2d_float_get_size(&height, &width, Iu);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matse3_check_size ( T );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matut3_check_size ( K );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size ( LtL, 8, 8 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size ( Lte, 8, 1 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_check_size ( Iv, height, width );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_check_size ( Iu, height, width );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_check_size ( Id, height, width );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_check_size ( Ia, height, width );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_check_size ( Im, height, width );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Buffer accessors

   Rox_Double * Lte_data = NULL;
   error = rox_array2d_double_get_data_pointer ( &Lte_data, Lte );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** LtL_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &LtL_data, LtL );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** K_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &K_data, K );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint ** dm = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer( &dm, Im );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** Iu_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer( &Iu_data, Iu );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** Iv_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer( &Iv_data, Iv );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** dd = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer( &dd, Id );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** da = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer( &da, Ia );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Calibration constants
   Rox_Float fu = (Rox_Float) K_data[0][0];
   Rox_Float fv = (Rox_Float) K_data[1][1];
   Rox_Float cu = (Rox_Float) K_data[0][2];
   Rox_Float cv = (Rox_Float) K_data[1][2];

   Rox_Float ifu = (Rox_Float) (1.0f / fu);
   Rox_Float ifv = (Rox_Float) (1.0f / fv);
   Rox_Float icu = (Rox_Float) (-cu / fu);
   Rox_Float icv = (Rox_Float) (-cv / fv);

   // Plane depth prepare
   Rox_Float pa = (Rox_Float) (-a / d);
   Rox_Float pb = (Rox_Float) (-b / d);
   Rox_Float pc = (Rox_Float) (-c / d);

   Rox_Float dizu = pa * ifu;
   Rox_Float dizv = pb * ifv;

   // Get pose information as required by jacobian
   Rox_Double ** T_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &T_data, T);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float er11 = (Rox_Float) T_data[0][0]; Rox_Float er12 = (Rox_Float) T_data[0][1]; Rox_Float er13 = (Rox_Float) T_data[0][2]; Rox_Float etx = (Rox_Float) T_data[0][3];
   Rox_Float er21 = (Rox_Float) T_data[1][0]; Rox_Float er22 = (Rox_Float) T_data[1][1]; Rox_Float er23 = (Rox_Float) T_data[1][2]; Rox_Float ety = (Rox_Float) T_data[1][3];
   Rox_Float er31 = (Rox_Float) T_data[2][0]; Rox_Float er32 = (Rox_Float) T_data[2][1]; Rox_Float er33 = (Rox_Float) T_data[2][2]; Rox_Float etz = (Rox_Float) T_data[2][3];

   Rox_Float taux = er11 * etx + ety * er21 + etz * er31;
   Rox_Float tauy = er12 * etx + ety * er22 + etz * er32;
   Rox_Float tauz = er13 * etx + er23 * ety + er33 * etz;

   // Zeroing jacobians
   error = rox_array2d_double_fillval ( LtL, 0.0 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillval ( Lte, 0.0 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint i = 0; i < height; i++)
   {
      Rox_Float vr = (float) (i);
      Rox_Float y = ifv * vr + icv;
      Rox_Float riz = pb * y + pc;

      for (Rox_Sint j = 0; j < width; j++)
      {
         Rox_Float L[6];

         if (!dm[i][j]) continue;

         Rox_Float ur = (float)(j);
         Rox_Float x = ifu * ur + icu;
         Rox_Float iz = pa * x + riz;

         // Retrieve per pixel params
         Rox_Float Iu = Iu_data[i][j];
         Rox_Float Iv = Iv_data[i][j];
         Rox_Float dif = dd[i][j];
         Rox_Float avg = da[i][j];

         Rox_Float t1 = (Rox_Float) (tauy - tauz * y);
         Rox_Float t2 = (Rox_Float) (1.0 + tauz * iz);
         Rox_Float t3 = (Rox_Float) (t1 * dizv + t2 * ifv);
         Rox_Float t4 = (Rox_Float) (taux - tauz * x);
         Rox_Float t5 = (Rox_Float) (t4 * dizu);
         Rox_Float t6 = (Rox_Float) (t5 * ifv + t3 * ifu);

         t6 = (Rox_Float) (1.0 / t6);
         t6 = t2 * t6;
         t1 = t6 * (Iu * t3 - Iv * t1 * dizu);
         t3 = t6 * (Iv * (t5 + t2 * ifu) - Iu * t4 * dizv);
         t2 = (Rox_Float) (1.0 / t2);
         t4 = - (t2 * t2 * (t1 * (x + taux * iz) + t3 * (y + tauy * iz)));
         t3 = t3 * t2;
         t1 = t1 * t2;

         L[0] = t1 * iz;
         L[1] = t3 * iz;
         L[2] = t4 * iz;
         L[3] = t4 * y - t3;
         L[4] = t1 - t4 * x;
         L[5] = t3 * x - t1 * y;

         // Update system
         for (Rox_Sint k = 0; k < 6; k++)
         {
            for (Rox_Sint l = 0; l <= k; l++)
            {
               LtL_data[k][l] += L[k] * L[l];
            }

            Lte_data[k] += L[k] * dif;
         }

         LtL_data[6][0] += L[0] * avg;
         LtL_data[6][1] += L[1] * avg;
         LtL_data[6][2] += L[2] * avg;
         LtL_data[6][3] += L[3] * avg;
         LtL_data[6][4] += L[4] * avg;
         LtL_data[6][5] += L[5] * avg;
         LtL_data[6][6] += avg * avg;

         LtL_data[7][0] += L[0];
         LtL_data[7][1] += L[1];
         LtL_data[7][2] += L[2];
         LtL_data[7][3] += L[3];
         LtL_data[7][4] += L[4];
         LtL_data[7][5] += L[5];
         LtL_data[7][6] += avg;
         LtL_data[7][7] += 1;

         Lte_data[6] += avg * dif;
         Lte_data[7] += dif;
      }
   }

   // Symmetrise lower triangular input
   error = rox_array2d_double_symmetrise_lower ( LtL );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}
