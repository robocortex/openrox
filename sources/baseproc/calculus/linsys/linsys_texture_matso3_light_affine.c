//==============================================================================
//
//    OPENROX   : File linsys_texture_matso3_light_affine.c
//
//    Contents  : Implementation of linsys_texture_matso3_light_affine module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "linsys_texture_matso3_light_affine.h"

#include <baseproc/maths/maths_macros.h>

#include <baseproc/array/fill/fillval.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_jacobian_so3_light_affine_premul (
   Rox_Matrix LtL,
   Rox_Matrix Lte,
   const Rox_Array2D_Float Iu,
   const Rox_Array2D_Float Iv,
   const Rox_Array2D_Float Ia,
   const Rox_Array2D_Float Id,
   const Rox_Imask Im,
   const Rox_MatSE3 pose,
   const Rox_MatUT3 K
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !LtL || !Lte )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( !Iu || !Iv || !Ia || !Id || !K || !pose || !Im )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_check_size(pose, 4, 4);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(K, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(LtL, 5, 5);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(Lte, 5, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint width = 0, height = 0;
   error = rox_array2d_float_get_size(&height, &width, Iu);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_check_size(Iv, height, width);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_check_size(Iu, height, width);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_check_size(Id, height, width);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_check_size(Ia, height, width);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_check_size ( Im, height, width );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** LtL_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &LtL_data, LtL );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double  * Lte_data = NULL;
   error = rox_array2d_double_get_data_pointer ( &Lte_data, Lte );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** K_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &K_data, K );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float fu = (Rox_Float) K_data[0][0];
   Rox_Float fv = (Rox_Float) K_data[1][1];
   Rox_Float cu = (Rox_Float) K_data[0][2];
   Rox_Float cv = (Rox_Float) K_data[1][2];

   Rox_Double ** dT = NULL;
   // Get pose information as required by jacobian
   error = rox_array2d_double_get_data_pointer_to_pointer( &dT, pose );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float r1 = (Rox_Float) dT[0][0];
   Rox_Float r2 = (Rox_Float) dT[0][1];
   Rox_Float r3 = (Rox_Float) dT[0][2];
   Rox_Float r4 = (Rox_Float) dT[1][0];
   Rox_Float r5 = (Rox_Float) dT[1][1];
   Rox_Float r6 = (Rox_Float) dT[1][2];
   Rox_Float r7 = (Rox_Float) dT[2][0];
   Rox_Float r8 = (Rox_Float) dT[2][1];
   Rox_Float r9 = (Rox_Float) dT[2][2];
   Rox_Float tx = (Rox_Float) dT[0][3];
   Rox_Float ty = (Rox_Float) dT[1][3];
   Rox_Float tz = (Rox_Float) dT[2][3];

   Rox_Uint **Im_data = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer ( &Im_data, Im );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float **Iu_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &Iu_data, Iu );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** Iv_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &Iv_data, Iv );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** Id_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &Id_data, Id );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** Ia_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &Ia_data, Ia );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillval ( LtL, 0 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillval ( Lte, 0 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint i = 0; i < height; i++)
   {
      Rox_Float vr = (float) (i);

      for (Rox_Sint j = 0; j < width; j++)
      {
         Rox_Float L[3];

         if (!Im_data[i][j]) continue;

         Rox_Float ur = (float)(j);

         // Retrieve per pixel params
         Rox_Float Iu_val = Iu_data[i][j];
         Rox_Float Iv_val = Iv_data[i][j];
         Rox_Float d = Id_data[i][j];
         Rox_Float a = Ia_data[i][j];

         // Jacobian row
         //         L[0] = (Iv_val * ((-r9 * r5 + (r6 + ty) * r8 - r5 * tz) * fu + (ur - cu) * (r8 * r4 - r5 * r7)) * pow((double) fv, 0.3e1) + (Iu * (-r9 * r2 + (r3 + tx) * r8 - r2 * tz) * fu * fu + (-Iu_val * (ur - cu) * (r2 * r7 - r8 * r1) + Iv_val * (vr - cv) * (-r9 * ty + r6 * tz)) * fu - Iv_val * (vr - cv) * (ur - cu) * (r9 * r4 - r6 * r7)) * fv * fv + (-Iu_val * (-r3 * tz + r9 * tx) * fu + (ur - cu) * (r3 * r7 - r9 * r1) * Iu_val - Iv_val * (vr - cv) * (-r6 * r8 + r9 * r5)) * fu * (vr - cv) * fv + Iu_val * fu * fu * pow((double)(vr - cv), 0.2e1) * (-r9 * r2 + r3 * r8)) * fu * pow((double)((r9 + tz) * fu + r7 * (ur - cu)) * fv + r8 * fu * (vr - cv), -0.2e1);
         //         L[1] = -fv * (((-r9 * r1 + (r3 + tx) * r7 - r1 * tz) * fv + (vr - cv) * (r2 * r7 - r8 * r1)) * Iu_val * pow((double) fu, 0.3e1) + (-Iv * (r9 * r4 + (-ty - r6) * r7 + r4 * tz) * fv * fv + (-(ur - cu) * (-r3 * tz + r9 * tx) * Iu_val - Iv_val * (vr - cv) * (r8 * r4 - r5 * r7)) * fv + Iu_val * (vr - cv) * (ur - cu) * (-r9 * r2 + r3 * r8)) * fu * fu + (ur - cu) * fv * (Iv_val * (-r9 * ty + r6 * tz) * fv + (ur - cu) * (r3 * r7 - r9 * r1) * Iu_val - Iv_val * (vr - cv) * (-r6 * r8 + r9 * r5)) * fu - Iv_val * fv * fv * pow((double)(ur - cu), 0.2e1) * (r9 * r4 - r6 * r7)) * pow((double)((r9 + tz) * fv + r8 * (vr - cv)) * fu + r7 * fv * (ur - cu), -0.2e1);
         //         L[2] = ((((r3 + tx) * r7 - r1 * (r9 + tz)) * fv + (vr - cv) * (r2 * r7 - r8 * r1)) * Iu_val * (vr - cv) * pow((double) fu, 0.3e1) - fv * (((ur - cu) * ((r3 + tx) * r8 - r2 * (r9 + tz)) * Iu_val + Iv_val * ((-ty - r6) * r7 + r4 * (r9 + tz)) * (vr - cv)) * fv + Iv_val * pow((double)(vr - cv), 0.2e1) * (r8 * r4 - r5 * r7)) * fu * fu + (((-ty - r6) * r8 + r5 * (r9 + tz)) * Iv_val * fv + Iu_val * (ur - cu) * (r2 * r7 - r8 * r1)) * (ur - cu) * fv * fv * fu - Iv_val * pow((double) fv, 0.3e1) * pow((double)(ur - cu), 0.2e1) * (r8 * r4 - r5 * r7)) * pow((double)((r9 + tz) * fv + r8 * (vr - cv)) * fu + r7 * fv * (ur - cu), -0.2e1);
         L[0] = (Rox_Float)((Iv_val * (( -r9 * r5 + (r6 + ty) * r8 - r5 * tz) * fu + (ur - cu) * (r8 * r4 - r5 * r7)) * pow(fv, 0.3e1f) + (Iu_val * (-r9 * r2 + (r3 + tx) * r8 - r2 * tz) * fu * fu + (-Iu_val * (ur - cu) * (r2 * r7 - r8 * r1) + Iv_val * (vr - cv) * (-r9 * ty + r6 * tz)) * fu - Iv_val * (vr - cv) * (ur - cu) * (r9 * r4 - r6 * r7)) * fv * fv + (-Iu_val * (-r3 * tz + r9 * tx) * fu + (ur - cu) * (r3 * r7 - r9 * r1) * Iu_val - Iv_val * (vr - cv) * (-r6 * r8 + r9 * r5)) * fu * (vr - cv) * fv + Iu_val * fu * fu * pow(vr - cv, 0.2e1f) * (-r9 * r2 + r3 * r8)) * fu * pow(((r9 + tz) * fu + r7 * (ur - cu)) * fv + r8 * fu * (vr - cv), -0.2e1f));
         L[1] = (Rox_Float)(-fv * (((-r9 * r1 + (r3 + tx) * r7 - r1 * tz) * fv + (vr - cv) * (r2 * r7 - r8 * r1)) * Iu_val * pow(fu, 0.3e1f) + (-Iv_val * (r9 * r4 + (-ty - r6) * r7 + r4 * tz) * fv * fv + (-(ur - cu) * (-r3 * tz + r9 * tx) * Iu_val - Iv_val * (vr - cv) * (r8 * r4 - r5 * r7)) * fv + Iu_val * (vr - cv) * (ur - cu) * (-r9 * r2 + r3 * r8)) * fu * fu + (ur - cu) * fv * (Iv_val * (-r9 * ty + r6 * tz) * fv + (ur - cu) * (r3 * r7 - r9 * r1) * Iu_val - Iv_val * (vr - cv) * (-r6 * r8 + r9 * r5)) * fu - Iv_val * fv * fv * pow(ur - cu, 0.2e1f) * (r9 * r4 - r6 * r7)) * pow(((r9 + tz) * fv + r8 * (vr - cv)) * fu + r7 * fv * (ur - cu), -0.2e1f));
         L[2] = (Rox_Float) (((((r3 + tx) * r7 - r1 * (r9 + tz)) * fv + (vr - cv) * (r2 * r7 - r8 * r1)) * Iu_val * (vr - cv) * pow(fu, 0.3e1f) - fv * (((ur - cu) * ((r3 + tx) * r8 - r2 * (r9 + tz)) * Iu_val + Iv_val * ((-ty - r6) * r7 + r4 * (r9 + tz)) * (vr - cv)) * fv + Iv_val * pow(vr - cv, 0.2e1f) * (r8 * r4 - r5 * r7)) * fu * fu + (((-ty - r6) * r8 + r5 * (r9 + tz)) * Iv_val * fv + Iu_val * (ur - cu) * (r2 * r7 - r8 * r1)) * (ur - cu) * fv * fv * fu - Iv_val * pow(fv, 0.3e1f) * pow(ur - cu, 0.2e1f) * (r8 * r4 - r5 * r7)) * pow(((r9 + tz) * fv + r8 * (vr - cv)) * fu + r7 * fv * (ur - cu), -0.2e1f));

         // Update system
         for (Rox_Sint k = 0; k < 3; k++)
         {
            for (Rox_Sint l = 0; l <= k; l++)
            {
               LtL_data[k][l] += L[k] * L[l];
            }

            Lte_data[k] += L[k] * d;
         }

         LtL_data[3][0] += L[0] * a;
         LtL_data[3][1] += L[1] * a;
         LtL_data[3][2] += L[2] * a;
         LtL_data[3][3] += a * a;

         LtL_data[4][0] += L[0];
         LtL_data[4][1] += L[1];
         LtL_data[4][2] += L[2];
         LtL_data[4][3] += a;
         LtL_data[4][4] += 1;

         Lte_data[3] += a * d;
         Lte_data[4] += d;
      }
   }

   // Symmetrise
   for (Rox_Sint k = 0; k < 5; k++)
   {
      for (Rox_Sint l = k; l < 5; l++)
      {
         LtL_data[k][l] = LtL_data[l][k];
      }
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_jacobian_so3_simple_light_affine_premul (
   Rox_Matrix LtL,
   Rox_Matrix Lte,
   Rox_Array2D_Float Iu,
   Rox_Array2D_Float Iv,
   Rox_Array2D_Float Ia,
   Rox_Array2D_Float Id,
   Rox_Imask Im,
   Rox_Array2D_Double K
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Double **K_data = NULL, **LtL_data = NULL, *Lte_data = NULL;
   Rox_Uint **Im_data = NULL;

   if (!LtL || !Lte)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!Iu || !Iv || !Ia || !Id || !K || !Im)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_matut3_check_size ( K );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(LtL, 5, 5);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(Lte, 5, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_float_get_size ( &rows, &cols, Iu );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_check_size ( Iv, rows, cols );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_check_size ( Iu, rows, cols );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_check_size ( Id, rows, cols );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_check_size ( Ia, rows, cols );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_check_size ( Im, rows, cols );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer_to_pointer ( &K_data, K );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer_to_pointer ( &LtL_data, LtL );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer ( &Lte_data, Lte );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float fu = (Rox_Float) K_data[0][0];
   Rox_Float fv = (Rox_Float) K_data[1][1];
   Rox_Float cu = (Rox_Float) K_data[0][2];
   Rox_Float cv = (Rox_Float) K_data[1][2];

   // ifu = 1.0 / fu;
   // ifv = 1.0 / fv;
   // icu = - cu * ifu;
   // icv = - cv * ifv;

   error  = rox_array2d_uint_get_data_pointer_to_pointer( &Im_data, Im );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** Iu_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer( &Iu_data, Iu );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** Iv_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer( &Iv_data, Iv );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** Id_data = NULL;
   error  = rox_array2d_float_get_data_pointer_to_pointer( &Id_data, Id );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** Ia_data = NULL;
   error  = rox_array2d_float_get_data_pointer_to_pointer( &Ia_data, Ia );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error  = rox_array2d_double_fillval ( LtL, 0 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error  = rox_array2d_double_fillval ( Lte, 0 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint i = 0; i < rows; i++)
   {
      Rox_Float v = (float)(i);

      for (Rox_Sint j = 0; j < cols; j++)
      {
         Rox_Float L[3];

         if (!Im_data[i][j]) continue;

         Rox_Float u = (float)(j);

         // Retrieve per pixel params
         Rox_Float Iu = Iu_data[i][j];
         Rox_Float Iv = Iv_data[i][j];
         Rox_Float d = Id_data[i][j];
         Rox_Float a = Ia_data[i][j];

         // Jacobian row
         L[0] = (Rox_Float) ((-(v - cv) * (u - cu) * Iu - Iv * (v * v - 2.0 * cv * v + cv * cv + fv * fv)) / fv);
         L[1] = (Rox_Float) (((u * u - 2.0 * cu * u + cu * cu + fu * fu) * Iu + Iv * (v - cv) * (u - cu)) / fu);
         L[2] = (Rox_Float) ((Iv * fv * fv * (u - cu) - Iu * fu * fu * (v - cu)) / fu / fv);

         // rewrite 
         //L[0] = - y * ( fu * Iu * x + fv * Iv * y );
         //L[1] = + x * ( fu * Iu * x + fv * Iv * y );
         //L[2] =       ( fv * Iv * x - fu * Iu * y );

         // Can be obtained from SE3 jacobian supposing otc = [0;0;-1]

         // Update system
         for (Rox_Uint k = 0; k < 3; k++)
         {
            for (Rox_Uint l = 0; l <= k; l++)
            {
               LtL_data[k][l] += L[k] * L[l];
            }

            Lte_data[k] += L[k] * d;
         }

         LtL_data[3][0] += L[0] * a;
         LtL_data[3][1] += L[1] * a;
         LtL_data[3][2] += L[2] * a;
         LtL_data[3][3] += a * a;

         LtL_data[4][0] += L[0];
         LtL_data[4][1] += L[1];
         LtL_data[4][2] += L[2];
         LtL_data[4][3] += a;
         LtL_data[4][4] += 1;

         Lte_data[3] += a * d;
         Lte_data[4] += d;
      }
   }

   // Symmetrise
   for (Rox_Uint k = 0; k < 5; k++)
   {
      for (Rox_Uint l = k; l < 5; l++)
      {
         LtL_data[k][l] = LtL_data[l][k];
      }
   }

function_terminate:
   return error;
}
