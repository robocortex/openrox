//============================================================================
//
//    OPENROX   : File linsys_point2d_nor_matse3_weighted.c
//
//    Contents  : Implementation of linsys_point2d_nor_matse3_weighted module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//============================================================================

#include "linsys_point2d_nor_matse3_weighted.h"

#include <generated/dynvec_point3d_float_struct.h>
#include <baseproc/geometry/point/point3d.h>

#include <baseproc/array/fill/fillval.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_jacobian_se3_from_points_weighted_premul_float (
   Rox_Matrix LtL, 
   Rox_Matrix Lte, 
   const Rox_Array2D_Double diff, 
   const Rox_Array2D_Double weight, 
   const Rox_DynVec_Point3D_Float meters
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !LtL || !Lte )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( !diff || !weight || !meters) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint size = meters->used;
   error = rox_array2d_double_check_size(LtL, 6, 6); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(Lte, 6, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(diff, size * 2, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(weight, size, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Point3D_Float dm = meters->data;

   Rox_Double **LtL_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &LtL_data, LtL);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double *Lte_data = NULL;
   error = rox_array2d_double_get_data_pointer ( &Lte_data, Lte);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double * dd = NULL;
   error = rox_array2d_double_get_data_pointer ( &dd, diff);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double * dw = NULL;
   error = rox_array2d_double_get_data_pointer ( &dw, weight);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillval(LtL, 0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillval(Lte, 0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint i = 0; i < size; i++)
   {
      Rox_Double L1[6], L2[6];

      Rox_Double x = dm[i].X / dm[i].Z;
      Rox_Double y = dm[i].Y / dm[i].Z;
      Rox_Double Z = dm[i].Z;
      // TODO: test if Z is 0
      Rox_Double zi = 1.0 / Z;
      // Get weigth value
      Rox_Double w = dw[i];

      // Get weigthed differences
      Rox_Double d1 = w * dd[i * 2];
      Rox_Double d2 = w * dd[i * 2 + 1];

      // Translation : 1st row 
      L1[0] = -zi * w ;
      L1[1] = 0.0 ;
      L1[2] = +zi * x * w;

      // Rotation : 1st row 
      L1[3] = (x * y) * w;
      L1[4] = (- x * x - 1.0) * w;
      L1[5] = + y * w ;

      // Translation : 2nd row 
      L2[0] =  0.0 ;
      L2[1] = -zi * w ;
      L2[2] = +zi * y * w;

      // Rotation : 2nd row 
      L2[3] = (+ y * y + 1.0) * w;
      L2[4] = - x * y * w;
      L2[5] = - x * w;

      for (Rox_Sint k = 0; k < 6; k++)
      {
         for (Rox_Sint l = 0; l <= k; l++)
         {
            LtL_data[k][l] += L1[k] * L1[l] + L2[k] * L2[l];
         }

         Lte_data[k] += L1[k] * d1 + L2[k] * d2;
      }
   }

   for (Rox_Sint k = 0; k < 6; k++)
   {
      for (Rox_Sint l = k; l < 6; l++)
      {
         LtL_data[k][l] = LtL_data[l][k];
      }
   }

function_terminate:
   return error;
}
