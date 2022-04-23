//==============================================================================
//
//    OPENROX   : File linsys_stereo_point2d_pix_matse3_weighted.c
//
//    Contents  : Implementation of linsys_stereo_point2d_pix_matse3_weighted module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "linsys_stereo_point2d_pix_matse3_weighted.h"

#include <generated/dynvec_point3d_float_struct.h>

#include <baseproc/maths/maths_macros.h>
#include <baseproc/geometry/point/point3d.h>
#include <baseproc/array/fill/fillval.h>
#include <baseproc/array/symmetrise/symmetrise.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_jacobian_se3_from_stereo_points_pixels_weighted_premul_float(
   Rox_Matrix LtL, 
   Rox_Matrix Lte, 
   Rox_Array2D_Double diff, 
   Rox_Array2D_Double weight, 
   Rox_DynVec_Point3D_Float meters, 
   Rox_Array2D_Double calib, 
   Rox_Array2D_Double pose, 
   Rox_Array2D_Double rTl
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !LtL || !Lte )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( !diff || !weight || !meters || !calib ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint size = meters->used;
   
   error = rox_array2d_double_check_size(calib, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_check_size(LtL, 6, 6);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_check_size(Lte, 6, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_check_size(diff, size * 2, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_check_size(weight, size, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Point3D_Float dm = meters->data;

   Rox_Double **dk;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &dk, calib);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double **drtl;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &drtl, rTl);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double **dt;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &dt, pose);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double **LtL_data;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &LtL_data, LtL);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double * Lte_data = NULL;
   error = rox_array2d_double_get_data_pointer ( &Lte_data, Lte );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double *dd = NULL;
   error = rox_array2d_double_get_data_pointer ( &dd, diff );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Double *dw = NULL;
   error = rox_array2d_double_get_data_pointer ( &dw, weight );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double dx = drtl[0][3];

   Rox_Double fu = dk[0][0];
   Rox_Double fv = dk[1][1];
   Rox_Double cu = dk[0][2];
   Rox_Double cv = dk[1][2];

   error = rox_array2d_double_fillval(LtL, 0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillval(Lte, 0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double m1 = (double) fu * dt[0][0] + (double) cu * dt[2][0];
   Rox_Double m2 = (double) fu * dt[0][1] + (double) cu * dt[2][1];
   Rox_Double m3 = (double) fu * dt[0][2] + (double) cu * dt[2][2];
   Rox_Double m4 = fu * dt[0][3] + fu * dx + cu * dt[2][3];
   Rox_Double m5 = (double) fv * dt[1][0] + (double) cv * dt[2][0];
   Rox_Double m6 = (double) fv * dt[1][1] + (double) cv * dt[2][1];
   Rox_Double m7 = (double) fv * dt[1][2] + (double) cv * dt[2][2];
   Rox_Double m8 = fv * dt[1][3] + cv * dt[2][3];
   Rox_Double m9 = dt[2][0];
   Rox_Double m10 = dt[2][1];
   Rox_Double m11 = dt[2][2];
   Rox_Double m12 = dt[2][3];

   for (Rox_Sint i = 0; i < size; i++)
   {
      Rox_Double L1[6], L2[6];

      Rox_Double X = dm[i].X;
      Rox_Double Y = dm[i].Y;
      Rox_Double Z = dm[i].Z;

      Rox_Double w = dw[i];
      Rox_Double d1 = w * dd[i * 2];
      Rox_Double d2 = w * dd[i * 2 + 1];

      Rox_Double x = m1 * X + m2 * Y + m3 * Z + m4;
      Rox_Double y = m5 * X + m6 * Y + m7 * Z + m8;
      Rox_Double z = m9 * X + m10 * Y + m11 * Z + m12;

      Rox_Double t1 = 1.0 / z;
      Rox_Double t2 = t1 * x;
      Rox_Double t3 = -m11 * Y + m10 * Z;
      Rox_Double t4 = m11 * X - m9 * Z;
      Rox_Double t5 = -m10 * X + m9 * Y;
      Rox_Double t6 = t1 * y;

      t1 = w / z;

      L1[0] = t1 * (m1 - t2 * m9);
      L1[1] = t1 * (m2 - t2 * m10);
      L1[2] = t1 * (m3 - t2 * m11);
      L1[3] = t1 * (t2 * t3 + m3 * Y - m2 * Z);
      L1[4] = t1 * (t2 * t4 - m3 * X + m1 * Z);
      L1[5] = t1 * (t2 * t5 + m2 * X - m1 * Y);
      L2[0] = t1 * (m5 - t6 * m9);
      L2[1] = t1 * (m6 - t6 * m10);
      L2[2] = t1 * (m7 - t6 * m11);
      L2[3] = t1 * (t6 * t3 + m7 * Y - m6 * Z);
      L2[4] = t1 * (t6 * t4 - m7 * X + m5 * Z);
      L2[5] = t1 * (t6 * t5 + m6 * X - m5 * Y);

      for (Rox_Sint k = 0; k < 6; k++)
      {
         for (Rox_Sint l = 0; l <= k; l++)
         {
            LtL_data[k][l] += L1[k] * L1[l] + L2[k] * L2[l];
         }

         Lte_data[k] -= L1[k] * d1 + L2[k] * d2;
      }
   }

   // Symmetrise lower triangular input
   error = rox_array2d_double_symmetrise_lower ( LtL );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}
