//==============================================================================
//
//    OPENROX   : File dynvec_points2d_tools.c
//
//    Contents  : Implementation of dynvec points2d tools module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "dynvec_point2d_tools.h"

#include <float.h>

#include <generated/dynvec_point2d_double_struct.h>
#include <generated/dynvec_point2d_float_struct.h>

#include <baseproc/maths/maths_macros.h>
#include <baseproc/geometry/point/point2d.h>

#include <inout/system/errors_print.h>

#include <system/errors/errors.h>

#define THRESH_NUMERICAL_ZERO 1e-16

Rox_ErrorCode rox_dynvec_point2d_double_set_data (
   Rox_DynVec_Point2D_Double points2D,
   Rox_Double * data_points2D,
   Rox_Uint numb_points2D
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Point2D_Double_Struct point2D;

   // TODO check inputs

   for ( Rox_Uint i=0; i<numb_points2D; i++)
   {
      point2D.u = data_points2D[2*i];
      point2D.v = data_points2D[2*i+1];
      error = rox_dynvec_point2d_double_append(points2D, &point2D);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   return error;
}


Rox_ErrorCode rox_dynvec_point2d_float_set_data (
   Rox_DynVec_Point2D_Float points2D,
   Rox_Float * data_points2D,
   Rox_Uint numb_points2D
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Point2D_Float_Struct point2D;

   // TODO check inputs

   for ( Rox_Uint i=0; i<numb_points2D; i++)
   {
      point2D.u = data_points2D[2*i];
      point2D.v = data_points2D[2*i+1];
      error = rox_dynvec_point2d_float_append(points2D, &point2D);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   return error;
}


Rox_ErrorCode rox_dynvec_point2d_float_append_vector_point2d_float (
   Rox_DynVec_Point2D_Float points2D,
   Rox_Point2D_Double points2D_double,
   Rox_Uint numb_points2D
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Point2D_Float_Struct point2D_struct_float;

   // TODO check inputs

   for ( Rox_Uint i=0; i<numb_points2D; i++)
   {
      point2D_struct_float.u = (Rox_Float) points2D_double[i].u;
      point2D_struct_float.v = (Rox_Float) points2D_double[i].v;
      error = rox_dynvec_point2d_float_append(points2D, &point2D_struct_float);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   return error;
}


Rox_ErrorCode rox_dynvec_point2d_double_append_vector_point2d_double (
   Rox_DynVec_Point2D_Double dynvec_points2D,
   Rox_Point2D_Double points2D,
   Rox_Uint numb_points2D)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   // TODO check inputs

   for (Rox_Uint i=0; i<numb_points2D; i++)
   {
      error = rox_dynvec_point2d_double_append(dynvec_points2D, &points2D[i]);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   return error;
}


Rox_ErrorCode rox_dynvec_point2d_double_transform_homogeneous (
   Rox_DynVec_Point2D_Double result,
   Rox_DynVec_Point2D_Double source,
   Rox_Array2D_Double             A
)
{
   Rox_ErrorCode   error = ROX_ERROR_NONE;
   Rox_Double      **A_data = NULL;
   Rox_Double      A_00, A_01, A_02, A_10, A_11, A_12, A_20, A_21, A_22;
   Rox_Double      denom, i_denom;

   if ( NULL == result || NULL == source || NULL == A )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_check_size( A, 3, 3 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer_to_pointer( &A_data, A );
   ROX_ERROR_CHECK_TERMINATE ( error );

   A_00 = A_data[0][0]; A_01 = A_data[0][1]; A_02 = A_data[0][2];
   A_10 = A_data[1][0]; A_11 = A_data[1][1]; A_12 = A_data[1][2];
   A_20 = A_data[2][0]; A_21 = A_data[2][1]; A_22 = A_data[2][2];

   result->used = 0;
   error = rox_dynvec_point2d_double_usecells( result, source->used );
   ROX_ERROR_CHECK_TERMINATE ( error );

   for ( Rox_Uint ii = 0; ii < source->used; ii++ )
   {
      Rox_Double su = source->data[ii].u;
      Rox_Double sv = source->data[ii].v;

      denom = A_20*su + A_21*sv + A_22;
      if ( fabs( denom ) < THRESH_NUMERICAL_ZERO )
      { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE( error ); }
      i_denom = 1.0 / denom;

      result->data[ii].u = ( A_00*su + A_01*sv + A_02 ) * i_denom;
      result->data[ii].v = ( A_10*su + A_11*sv + A_12 ) * i_denom;
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_dynvec_point2d_convert_pixel_double_to_meter_double (
   Rox_DynVec_Point2D_Double point_nor,
   Rox_DynVec_Point2D_Double point_pix,
   Rox_Array2D_Double K
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !point_nor || !point_pix || !K )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double ** K_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &K_data, K );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double fu = K_data[0][0];
   Rox_Double fv = K_data[1][1];
   Rox_Double cu = K_data[0][2];
   Rox_Double cv = K_data[1][2];

   for ( Rox_Uint k = 0; k < point_nor->used; k++ )
   {
      point_nor->data[k].u = ( point_pix->data[k].u - cu ) / fu;
      point_nor->data[k].v = ( point_pix->data[k].v - cv ) / fv;
   }

function_terminate:
   return error;
}


Rox_ErrorCode rox_array2d_point2d_double_distance_point2d_double (
   Rox_Array2D_Double dist,
   const Rox_DynVec_Point2D_Double one,
   const Rox_DynVec_Point2D_Double two
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !dist || !one || !two )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Uint size = one->used;
   if (two->used != size)
   { error = ROX_ERROR_ARRAYS_NOT_MATCH; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint rows = 0;
   error = rox_array2d_double_get_rows(&rows, dist);
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (rows != size)
   { error = ROX_ERROR_ARRAYS_NOT_MATCH; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double * dd = NULL;
   error = rox_array2d_double_get_data_pointer ( &dd, dist );
   ROX_ERROR_CHECK_TERMINATE ( error );

   for ( Rox_Uint i = 0; i < size; i++)
   {
      // Use rox_point2d_double_distance function in points2d_tools.h
      Rox_Double du = one->data[i].u - two->data[i].u;
      Rox_Double dv = one->data[i].v - two->data[i].v;
      dd[i] = du * du + dv * dv;
      dd[i] = sqrt(dd[i]);
   }

function_terminate:
   return error;
}


Rox_ErrorCode rox_array2d_point2d_double_distance_point2d_float (
   Rox_Array2D_Double dist,
   const Rox_DynVec_Point2D_Float one,
   const Rox_DynVec_Point2D_Float two
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!dist || !one || !two)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Uint size = one->used;

   if (two->used != size)
   { error = ROX_ERROR_ARRAYS_NOT_MATCH; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint rows = 0;
   error = rox_array2d_double_get_rows(&rows, dist);
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (rows != size)
   { error = ROX_ERROR_ARRAYS_NOT_MATCH; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double * dd = NULL;
   error = rox_array2d_double_get_data_pointer ( &dd, dist );
   ROX_ERROR_CHECK_TERMINATE ( error );

   for ( Rox_Uint i = 0; i < size; i++)
   {
      // Use rox_point2d_float_distance function in points2d_tools.h
      Rox_Double du = one->data[i].u - two->data[i].u;
      Rox_Double dv = one->data[i].v - two->data[i].v;
      dd[i] = du * du + dv * dv;
      dd[i] = sqrt(dd[i]);
   }

function_terminate:
   return error;
}


Rox_ErrorCode rox_array2d_point2d_double_difference_point2d_double (
   Rox_Array2D_Double difference,
   const Rox_DynVec_Point2D_Double one,
   const Rox_DynVec_Point2D_Double two
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!difference || !one || !two)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Uint size = one->used;
   if (two->used != size)
   { error = ROX_ERROR_ARRAYS_NOT_MATCH; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint rows = 0;
   error = rox_array2d_double_get_rows(&rows, difference);
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (rows != 2 * size)
   { error = ROX_ERROR_ARRAYS_NOT_MATCH; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double * de = NULL;
   error = rox_array2d_double_get_data_pointer ( &de, difference );
   ROX_ERROR_CHECK_TERMINATE ( error );

   for ( Rox_Uint i = 0; i < size; i++)
   {
      Rox_Double du = one->data[i].u - two->data[i].u;
      Rox_Double dv = one->data[i].v - two->data[i].v;
      de[i * 2] = du;
      de[i * 2 + 1] = dv;
   }

function_terminate:
   return error;
}


Rox_ErrorCode rox_array2d_point2d_double_difference_point2d_float (
   Rox_Array2D_Double difference,
   const Rox_DynVec_Point2D_Float one,
   const Rox_DynVec_Point2D_Float two
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!difference || !one || !two)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Uint size = one->used;
   if (two->used != size)
   { error = ROX_ERROR_ARRAYS_NOT_MATCH; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint rows = 0;
   error = rox_array2d_double_get_rows(&rows, difference);
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (rows != 2 * size)
   { error = ROX_ERROR_ARRAYS_NOT_MATCH; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double * de = NULL;
   error = rox_array2d_double_get_data_pointer ( &de, difference );
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Uint i = 0; i < size; i++)
   {
      Rox_Double du = one->data[i].u - two->data[i].u;
      Rox_Double dv = one->data[i].v - two->data[i].v;
      de[i * 2] = du;
      de[i * 2 + 1] = dv;
   }

function_terminate:
   return error;
}
