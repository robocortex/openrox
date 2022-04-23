//==============================================================================
//
//    OPENROX   : File bundle_frame.c
//
//    Contents  : Implementation of bundle_frame module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "bundle_frame.h"
#include "bundle_frame_struct.h"
#include "bundle_point_struct.h"

#include <float.h>

#include <baseproc/maths/maths_macros.h>
#include <baseproc/array/fill/fillunit.h>
#include <baseproc/array/fill/fillval.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_bundle_frame_new(Rox_Bundle_Frame * obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Bundle_Frame ret = NULL;

   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *obj = NULL;

   ret = (Rox_Bundle_Frame)rox_memory_allocate(sizeof(*ret), 1);
   if (!ret) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret->pose = NULL;
   ret->pose_previous = NULL;
   ret->measures = NULL;
   ret->projected_error = NULL;
   ret->hessian = NULL;
   ret->is_fixed = 0;
   ret->is_invalid = 0;
   ret->pos = 0;

   error = rox_array2d_double_new(&ret->pose, 4, 4); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&ret->pose_previous, 4, 4); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&ret->hessian, 6, 6); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&ret->projected_error, 6, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_bundle_measure_new(&ret->measures, 10); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   rox_array2d_double_fillunit(ret->pose);

   *obj = ret;

function_terminate:
   if (error) rox_bundle_frame_del(&ret);
   return error;
}

Rox_ErrorCode rox_bundle_frame_del(Rox_Bundle_Frame * obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Bundle_Frame todel = NULL;

   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = * obj;
   *obj = NULL;

   if (!todel) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_dynvec_bundle_measure_del(&todel->measures);
   rox_array2d_double_del(&todel->pose);
   rox_array2d_double_del(&todel->pose_previous);
   rox_array2d_double_del(&todel->hessian);
   rox_array2d_double_del(&todel->projected_error);

   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_bundle_frame_compute_hessian(Rox_Bundle_Frame obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (obj->is_fixed) goto function_terminate;
   if (obj->is_invalid) goto function_terminate;

   Rox_Double ** dh = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dh, obj->hessian);

   Rox_Double *  de = NULL;
   error = rox_array2d_double_get_data_pointer( &de, obj->projected_error);

   rox_array2d_double_fillval(obj->hessian, 0);
   rox_array2d_double_fillval(obj->projected_error, 0);

   for (Rox_Uint idmes = 0; idmes < obj->measures->used; idmes++)
   {
      Rox_Bundle_Measure mes = obj->measures->data[idmes];
      if (mes->is_invalid) continue;

      for ( Rox_Sint i = 0; i < 6; i++)
      {
         for ( Rox_Sint j = 0; j < 6; j++)
         {
            dh[i][j] += mes->jacFrame[0][i] * mes->jacFrame[0][j];
            dh[i][j] += mes->jacFrame[1][i] * mes->jacFrame[1][j];
         }

         de[i] += mes->jacFrame[0][i] * mes->weight * mes->error_pixels.u;
         de[i] += mes->jacFrame[1][i] * mes->weight * mes->error_pixels.v;
      }
   }

function_terminate:
   return error;
}


