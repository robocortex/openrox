//==============================================================================
//
//    OPENROX   : File linsys_se3_light_affine_premul.c
//
//    Contents  : Implementation of linsys_se3z1_light_affine_premul module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "linsys_se3_light_affine_premul.h"
#include "ansi_linsys_se3_light_affine_premul.h"

#include <baseproc/array/fill/fillval.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_jacobian_se3_light_affine_premul ( 
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

   error = rox_matse3_check_size ( pose ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matut3_check_size ( K ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size ( LtL, 8, 8 ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size ( Lte, 8, 1 ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_float_get_size(&rows, &cols, Iu);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_check_size(Iv, rows, cols); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_check_size(Iu, rows, cols); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_check_size(Id, rows, cols); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_check_size(Ia, rows, cols); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_check_size(Im, rows, cols); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** K_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &K_data, K);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** LtL_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&LtL_data, LtL);
   ROX_ERROR_CHECK_TERMINATE ( error );
  
   Rox_Double * Lte_data = NULL;
   error = rox_array2d_double_get_data_pointer (&Lte_data, Lte);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float fu = K_data[0][0];
   Rox_Float fv = K_data[1][1];
   Rox_Float cu = K_data[0][2];
   Rox_Float cv = K_data[1][2];

   // Get pose information as required by jacobian
   Rox_Double ** T_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &T_data, pose );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float r11 = T_data[0][0]; Rox_Float r12 = T_data[0][1]; Rox_Float r13 = T_data[0][2]; Rox_Float tx = T_data[0][3];
   Rox_Float r21 = T_data[1][0]; Rox_Float r22 = T_data[1][1]; Rox_Float r23 = T_data[1][2]; Rox_Float ty = T_data[1][3];
   Rox_Float r31 = T_data[2][0]; Rox_Float r32 = T_data[2][1]; Rox_Float r33 = T_data[2][2]; Rox_Float tz = T_data[2][3];

   // Compute tau = otc = - inv(cRo) * cto
   Rox_Float taux = -(r11 * tx + r21 * ty + r31 * tz);
   Rox_Float tauy = -(r12 * tx + r22 * ty + r32 * tz);
   Rox_Float tauz = -(r13 * tx + r23 * ty + r33 * tz);

   Rox_Uint  ** Im_data  = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer( &Im_data, Im);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** Iu_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer(&Iu_data, Iu);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** Iv_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer(&Iv_data, Iv);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** Id_data  = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &Id_data, Id );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** Ia_data  = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &Ia_data, Ia );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillval(LtL, 0);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_fillval(Lte, 0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_ansi_linsys_se3_light_affine_premul ( LtL_data, Lte_data, K_data, T_data, Iu_data, Iv_data, Id_data, Ia_data, Im_data, rows, cols );
   ROX_ERROR_CHECK_TERMINATE ( error );

   if(0)
{
   for ( Rox_Sint i = 0; i < rows; i++)
   {
      Rox_Float L[6] = {0.0};

      Rox_Float vr = (Rox_Float)(i);
      Rox_Float y = (vr - cv)/fv;

      for ( Rox_Sint j = 0; j < cols; j++)
      {
         if (!Im_data[i][j]) continue;

         Rox_Float ur = (Rox_Float)(j);

         // Compute normalized coordinates
         Rox_Float x = (ur - cu)/fu;

         // Retrieve per pixel params
         Rox_Float Iu_val = Iu_data[i][j];
         Rox_Float Iv_val = Iv_data[i][j];
         Rox_Float d = Id_data[i][j];
         Rox_Float a = Ia_data[i][j];

         Rox_Float gv = Iv_val * fv;
         Rox_Float gu = Iu_val * fu;
         Rox_Float gw = (gu * (x - taux)  + gv * (y - tauy))/tauz;

         // Jacobian row
         // Translation
         L[0] = gu;
         L[1] = gv;
         L[2] = gw;
         // Rotation
         L[3] =   gw * y;
         L[4] = - gw * x;
         L[5] = x * gv - y * gu;

         // Update system
         for (Rox_Sint k = 0; k < 6; k++)
         {
            for (Rox_Sint l = 0; l <= k; l++)
            {
               LtL_data[k][l] += L[k] * L[l];
            }

            Lte_data[k] += L[k] * d;
         }

         LtL_data[6][0] += L[0] * a;
         LtL_data[6][1] += L[1] * a;
         LtL_data[6][2] += L[2] * a;
         LtL_data[6][3] += L[3] * a;
         LtL_data[6][4] += L[4] * a;
         LtL_data[6][5] += L[5] * a;
         LtL_data[6][6] += a * a;

         LtL_data[7][0] += L[0];
         LtL_data[7][1] += L[1];
         LtL_data[7][2] += L[2];
         LtL_data[7][3] += L[3];
         LtL_data[7][4] += L[4];
         LtL_data[7][5] += L[5];
         LtL_data[7][6] += a;
         LtL_data[7][7] += 1;

         Lte_data[6] += a * d;
         Lte_data[7] += d;
      }
   }

   // Symmetrise
   for (Rox_Sint k = 0; k < 8; k++)
   {
      for (Rox_Sint l = k; l < 8; l++)
      {
         LtL_data[k][l] = LtL_data[l][k];
      }
   }
}

function_terminate:
   return error;
}
