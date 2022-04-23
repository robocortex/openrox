//==============================================================================
//
//    OPENROX   : File bundle_point.c
//
//    Contents  : Implementation of bundle_point module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "bundle_point.h"
#include "bundle_point_struct.h"
#include "bundle_frame.h"

#include <float.h>

#include <baseproc/maths/maths_macros.h>
#include <baseproc/array/fill/fillval.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_bundle_point_new(Rox_Bundle_Point * obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Bundle_Point ret = NULL;

   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *obj = NULL;

   ret = (Rox_Bundle_Point) rox_memory_allocate(sizeof(*ret), 1);
   if (!ret) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret->measures = NULL;
   ret->hessian = NULL;
   ret->tp = NULL;
   ret->projected_error = NULL;
   ret->is_invalid = 0;

   ret->pos_jacobian = 0;
   ret->coords.X = 0;
   ret->coords.Y = 0;
   ret->coords.Z = 0;
   ret->coords_previous.X = 0;
   ret->coords_previous.Y = 0;
   ret->coords_previous.Z = 0;
   ret->update[0] = 0;
   ret->update[1] = 0;
   ret->update[2] = 0;

   error = rox_array2d_double_new(&ret->hessian, 3, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&ret->projected_error, 3, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&ret->tp, 3, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_bundle_measure_new(&ret->measures, 10); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   *obj = ret;

function_terminate:
   if (error) rox_bundle_point_del(&ret);
   return error;
}

Rox_ErrorCode rox_bundle_point_del(Rox_Bundle_Point * obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Bundle_Point todel = NULL;

   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *obj;
   *obj = NULL;

   if (!todel) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_dynvec_bundle_measure_del(&todel->measures);
   rox_array2d_double_del(&todel->projected_error);
   rox_array2d_double_del(&todel->hessian);
   rox_array2d_double_del(&todel->tp);

   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_bundle_point_compute_hessian(Rox_Bundle_Point obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double ** dh = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dh, obj->hessian);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double *  de = NULL;
   error = rox_array2d_double_get_data_pointer ( &de, obj->projected_error);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillval(obj->hessian, 0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillval(obj->projected_error, 0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Uint idmes = 0; idmes < obj->measures->used; idmes++)
   {
      Rox_Bundle_Measure mes = obj->measures->data[idmes];
      if (mes->is_invalid) continue;

      for ( Rox_Sint i = 0; i < 3; i++)
      {
         for ( Rox_Sint j = 0; j < 3; j++)
         {
            dh[i][j] += mes->jacPoint[0][i] * mes->jacPoint[0][j];
            dh[i][j] += mes->jacPoint[1][i] * mes->jacPoint[1][j];
         }

         de[i] += mes->jacPoint[0][i] * mes->weight * mes->error_pixels.u;
         de[i] += mes->jacPoint[1][i] * mes->weight * mes->error_pixels.v;
      }
   }

function_terminate:
   return error;
}

