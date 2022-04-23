//==============================================================================
//
//    OPENROX   : File linsys_point2d_pix_matsl3.c
//
//    Contents  : Implementation of linsys_point2d_pix_matsl3 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "linsys_point2d_pix_matsl3.h"

#include <baseproc/array/fill/fillval.h>
#include <generated/dynvec_point2d_float_struct.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_jacobian_sl3_from_points_weighted_premul_float (
   Rox_Matrix LtL, 
   Rox_Matrix Lte, 
   const Rox_Array2D_Double diff, 
   const Rox_Array2D_Double weight, 
   const Rox_DynVec_Point2D_Float ref
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !LtL || !Lte )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( !diff || !weight || !ref ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Uint size = ref->used;
   error = rox_array2d_double_check_size ( LtL, 8, 8 ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size ( Lte, 8, 1 ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size ( diff, size*2, 1 ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size ( weight, size, 1 ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Point2D_Float dp = ref->data;

   Rox_Double ** LtL_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &LtL_data, LtL);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double  * Lte_data = NULL;
   error = rox_array2d_double_get_data_pointer ( &Lte_data,  Lte);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double * dd = NULL;
   error = rox_array2d_double_get_data_pointer ( &dd,  diff);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double * dw = NULL;
   error = rox_array2d_double_get_data_pointer ( &dw,  weight);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillval(LtL, 0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillval(Lte, 0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Uint i = 0; i < size; i++)
   {
      Rox_Double L1[8], L2[8];

      Rox_Double u = dp[i].u;
      Rox_Double v = dp[i].v;
      Rox_Double w = dw[i];
      Rox_Double d1 = dd[i*2] * w;
      Rox_Double d2 = dd[i*2+1] * w;

      L1[0] = w;
      L1[1] = 0;
      L1[2] = w*v;
      L1[3] = 0;
      L1[4] = w*u;
      L1[5] = -w*u;
      L1[6] = -w*u*u;
      L1[7] = -w*u*v;

      L2[0] = 0;
      L2[1] = w;
      L2[2] = 0;
      L2[3] = w*u;
      L2[4] = -w*v;
      L2[5] = -2.0*w*v;
      L2[6] = -w*u*v;
      L2[7] = -w*v*v;

      for (Rox_Uint k = 0; k < 8; k++)
      {
         for (Rox_Uint l = 0; l <= k; l++)
         {
            LtL_data[k][l] += L1[k] * L1[l] + L2[k] * L2[l];
         }

         Lte_data[k] += L1[k] * d1 + L2[k] * d2;
      }
   }

   for (Rox_Uint k = 0; k < 8; k++)
   {
      for (Rox_Uint l = k; l < 8; l++)
      {
         LtL_data[k][l] = LtL_data[l][k];
      }

      Lte_data[k] = -Lte_data[k];
   }

function_terminate:
   return error;
}
