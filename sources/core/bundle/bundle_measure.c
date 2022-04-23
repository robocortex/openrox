//==============================================================================
//
//    OPENROX   : File bundle_measure.c
//
//    Contents  : Implementation of bundle_measure module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "bundle_measure.h"
#include "bundle_camera.h"
#include "bundle_frame.h"
#include "bundle_point.h"

#include "bundle_frame_struct.h"
#include "bundle_point_struct.h"

#include <float.h>

#include <baseproc/maths/maths_macros.h>
#include <baseproc/array/multiply/mulmatmat.h>

#include <baseproc/calculus/jacobians/jacobian_row_bundle_matse3_point3d.h>

#include <inout/system/errors_print.h>


Rox_ErrorCode rox_bundle_measure_new(Rox_Bundle_Measure * obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Bundle_Measure ret = NULL;

   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   *obj = NULL;

   ret = (Rox_Bundle_Measure)rox_memory_allocate(sizeof(*ret), 1);
   if (!ret) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret->camera = NULL;
   ret->frame = NULL;
   ret->point = NULL;
   ret->JpointTJframe = NULL;
   ret->is_invalid = 0;
   ret->Tpc = NULL;

   ret->weight = 1.0;

   error = rox_array2d_double_new(&ret->JpointTJframe, 3, 6);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&ret->Tpc, 6, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   *obj = ret;

function_terminate:
   if (error) rox_bundle_measure_del(&ret);

   return error;
}

Rox_ErrorCode rox_bundle_measure_del(Rox_Bundle_Measure * obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Bundle_Measure todel = NULL;

   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *obj;
   *obj = NULL;

   if (!todel) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_array2d_double_del(&todel->JpointTJframe);
   rox_array2d_double_del(&todel->Tpc);

   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_bundle_measure_predict(Rox_Bundle_Measure obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Point3D_Double_Struct buf;

   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (!obj->camera || !obj->frame || !obj->point)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double ** dk = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dk, obj->camera->calib);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Double ** dr = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dr, obj->camera->relative_pose);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dp = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dp, obj->frame->pose);
   ROX_ERROR_CHECK_TERMINATE ( error );

   buf.X = dp[0][0] * obj->point->coords.X + dp[0][1] * obj->point->coords.Y + dp[0][2] * obj->point->coords.Z + dp[0][3];
   buf.Y = dp[1][0] * obj->point->coords.X + dp[1][1] * obj->point->coords.Y + dp[1][2] * obj->point->coords.Z + dp[1][3];
   buf.Z = dp[2][0] * obj->point->coords.X + dp[2][1] * obj->point->coords.Y + dp[2][2] * obj->point->coords.Z + dp[2][3];

   obj->transformed_point.X = dr[0][0] * buf.X + dr[0][1] * buf.Y + dr[0][2] * buf.Z + dr[0][3];
   obj->transformed_point.Y = dr[1][0] * buf.X + dr[1][1] * buf.Y + dr[1][2] * buf.Z + dr[1][3];
   obj->transformed_point.Z = dr[2][0] * buf.X + dr[2][1] * buf.Y + dr[2][2] * buf.Z + dr[2][3];

   if (obj->transformed_point.Z < DBL_EPSILON)
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   obj->coords_estimated_meters.u = obj->transformed_point.X / obj->transformed_point.Z;
   obj->coords_estimated_meters.v = obj->transformed_point.Y / obj->transformed_point.Z;

   obj->coords_estimated_pixels.u = dk[0][0] * obj->coords_estimated_meters.u + dk[0][2];
   obj->coords_estimated_pixels.v = dk[1][1] * obj->coords_estimated_meters.v + dk[1][2];

   obj->error_pixels.u = obj->coords_estimated_pixels.u - obj->coords_pixels.u;
   obj->error_pixels.v = obj->coords_estimated_pixels.v - obj->coords_pixels.v;
   obj->distance_pixels = (obj->error_pixels.u*obj->error_pixels.u + obj->error_pixels.v*obj->error_pixels.v);

function_terminate:
   return error;
}

Rox_ErrorCode rox_bundle_measure_build_jacobians(Rox_Bundle_Measure obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double buf = NULL;

   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (!obj->camera || !obj->frame || !obj->point) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (obj->is_invalid) 
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_new(&buf, 4, 4);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_mulmatmat(buf, obj->camera->relative_pose, obj->frame->pose);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dt = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dt, buf);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dk = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dk, obj->camera->calib);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_jacobian_se3bundle_row_from_points_float(obj->jacFrame[0], obj->jacFrame[1], obj->jacPoint[0], obj->jacPoint[1], dt, &obj->transformed_point, &obj->point->coords);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Update with calib and weights
   for ( Rox_Sint i = 0; i < 6; i++)
   {
      obj->jacFrame[0][i] *= dk[0][0] * obj->weight;
      obj->jacFrame[1][i] *= dk[1][1] * obj->weight;
   }

   // Update with calib and weights
   for ( Rox_Sint i = 0; i < 3; i++)
   {
      obj->jacPoint[0][i] *= dk[0][0] * obj->weight;
      obj->jacPoint[1][i] *= dk[1][1] * obj->weight;
   }

   // Compute Jp'*Jc
   Rox_Double ** dh = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dh, obj->JpointTJframe);
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (obj->frame->is_fixed == 0)
   {
      for ( Rox_Sint i = 0; i < 3; i++)
      {
         for ( Rox_Sint j = 0; j < 6; j++)
         {
            dh[i][j] = obj->jacPoint[0][i] * obj->jacFrame[0][j];
            dh[i][j] += obj->jacPoint[1][i] * obj->jacFrame[1][j];
         }
      }
   }

function_terminate:
   rox_array2d_double_del(&buf);
   return error;
}
