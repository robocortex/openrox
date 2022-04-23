//==============================================================================
//
//    OPENROX   : File linsys_point2d_pix_matse3_weighted.c
//
//    Contents  : Implementation of linsys_point2d_pix_matse3_weighted module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "linsys_point2d_pix_matse3_weighted.h"

#include <baseproc/array/fill/fillval.h>
#include <baseproc/geometry/point/point3d_struct.h>
#include <baseproc/geometry/point/point3d.h>

#include <inout/system/errors_print.h>
#include <generated/dynvec_point3d_float_struct.h>
#include <generated/dynvec_point3d_double_struct.h>

Rox_ErrorCode rox_jacobian_se3_from_points_pixels_weighted_premul_float (
   Rox_Matrix                     LtL,
   Rox_Matrix                     Lte,
   const Rox_Array2D_Double             diff,
   const Rox_Array2D_Double             weight,
   const Rox_DynVec_Point3D_Float       meters,
   const Rox_Array2D_Double             calib )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !LtL || !Lte )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( !diff || !weight || !meters || !calib )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint size = meters->used;

   error = rox_array2d_double_check_size(    LtL,      6, 6 ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(    Lte,      6, 1 ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(  calib,      3, 3 ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(   diff, size*2, 1 ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size( weight,   size, 1 ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Point3D_Float dm = meters->data;

   Rox_Double **K_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &K_data, calib );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** LtL_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &LtL_data, LtL );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double  * Lte_data = NULL;
   error = rox_array2d_double_get_data_pointer ( &Lte_data, Lte );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double *dd = NULL;
   error = rox_array2d_double_get_data_pointer ( &dd, diff );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double *dw = NULL;
   error = rox_array2d_double_get_data_pointer ( &dw, weight );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double fu = K_data[0][0];
   Rox_Double fv = K_data[1][1];

   error = rox_array2d_double_fillval( LtL, 0 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillval( Lte, 0 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   for ( Rox_Sint i = 0; i < size; i++ )
   {
      Rox_Double L1[6], L2[6];

      Rox_Double x = dm[i].X / dm[i].Z;
      Rox_Double y = dm[i].Y / dm[i].Z;
      Rox_Double Z = dm[i].Z;
      Rox_Double zi = 1.0 / Z;

      Rox_Double w = dw[i];
      Rox_Double d1 = w * dd[i * 2];
      Rox_Double d2 = w * dd[i * 2 + 1];

      // Translation : 1st row
      L1[0] = fu * ( -zi * w );
      L1[1] = 0.0;
      L1[2] = fu * ( +zi * x * w );

      // Rotation : 1st row
      L1[3] = fu * ( x * y ) * w;
      L1[4] = fu * ( - x * x - 1.0 ) * w;
      L1[5] = fu * y * w ;

      // Translation : 2nd row
      L2[0] = 0.0;
      L2[1] = fv * ( -zi * w );
      L2[2] = fv * ( zi * y * w );

      // Rotation : 2nd row
      L2[3] = fv * ( + y * y + 1.0 ) * w;
      L2[4] = fv * ( - x * y * w );
      L2[5] = fv * ( - x * w );

      for ( Rox_Sint k = 0; k < 6; k++ )
      {
         for ( Rox_Sint l = 0; l <= k; l++ )
         {
            LtL_data[k][l] += L1[k] * L1[l] + L2[k] * L2[l];
         }

         Lte_data[k] += L1[k] * d1 + L2[k] * d2;
      }
   }

   for ( Rox_Sint k = 0; k < 6; k++ )
   {
      for ( Rox_Sint l = k; l < 6; l++ )
      {
         LtL_data[k][l] = LtL_data[l][k];
      }
   }

function_terminate:
   return error;
}


Rox_ErrorCode rox_jacobian_se3_from_points_pixels_weighted_premul_double(
  Rox_Array2D_Double               LtL,
  Rox_Array2D_Double               Lte,
  const Rox_Array2D_Double               diff,
  const Rox_Array2D_Double               weight,
  const Rox_DynVec_Point3D_Double        meters,
  const Rox_Array2D_Double               calib 
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !LtL || !Lte )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( !diff || !weight || !meters || !calib )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint size  = meters->used;

   error = rox_array2d_double_check_size(  calib,      3, 3 );     
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(    LtL,      6, 6 );     
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(    Lte,      6, 1 );     
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(   diff, size*2, 1 );     
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size( weight,   size, 1 );     
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Point3D_Double dm = meters->data;

   Rox_Double **K_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &K_data, calib );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** LtL_data = NULL;

   error = rox_array2d_double_get_data_pointer_to_pointer( &LtL_data,  LtL );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double  * Lte_data = NULL;
   error = rox_array2d_double_get_data_pointer ( &Lte_data,  Lte );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double *dd = NULL;
   error = rox_array2d_double_get_data_pointer ( &dd,  diff );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double *dw = NULL;
   error = rox_array2d_double_get_data_pointer ( &dw,  weight );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double fu = K_data[0][0];
   Rox_Double fv = K_data[1][1];

   error = rox_array2d_double_fillval( LtL, 0 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillval( Lte, 0 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   for ( Rox_Sint i = 0; i < size; i++ )
   {
      Rox_Double L1[6], L2[6];

      Rox_Double x = dm[i].X / dm[i].Z;
      Rox_Double y = dm[i].Y / dm[i].Z;
      Rox_Double Z = dm[i].Z;
      Rox_Double zi = 1.0 / Z;

      Rox_Double w = dw[i];
      Rox_Double d1 = w * dd[i * 2];
      Rox_Double d2 = w * dd[i * 2 + 1];

      // Translation : 1st row
      L1[0] = fu * ( -zi * w );
      L1[1] = 0.0;
      L1[2] = fu * ( +zi * x * w );

      // Rotation : 1st row
      L1[3] = fu * ( x * y ) * w;
      L1[4] = fu * ( - x * x - 1.0 ) * w;
      L1[5] = fu * y * w ;

      // Translation : 2nd row
      L2[0] = 0.0;
      L2[1] = fv * ( -zi * w );
      L2[2] = fv * ( zi * y * w );

      // Rotation : 2nd row
      L2[3] = fv * ( + y * y + 1.0 ) * w;
      L2[4] = fv * ( - x * y * w );
      L2[5] = fv * ( - x * w );

      for ( Rox_Sint k = 0; k < 6; k++ )
      {
         for ( Rox_Sint l = 0; l <= k; l++ )
         {
            LtL_data[k][l] += L1[k] * L1[l] + L2[k] * L2[l];
         }

         Lte_data[k] += L1[k] * d1 + L2[k] * d2;
      }
   }

   for ( Rox_Sint k = 0; k < 6; k++ )
   {
      for ( Rox_Sint l = k; l < 6; l++ )
      {
         LtL_data[k][l] = LtL_data[l][k];
      }
   }

function_terminate:
   return error;
}
