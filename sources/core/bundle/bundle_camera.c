//==============================================================================
//
//    OPENROX   : File bundle_camera.c
//
//    Contents  : Implementation of bundle_camera module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "bundle_camera.h"
#include <baseproc/array/fill/fillunit.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_bundle_camera_new(Rox_Bundle_Camera * obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Bundle_Camera ret = NULL;

   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *obj = NULL;

   ret = (Rox_Bundle_Camera) rox_memory_allocate(sizeof(*ret), 1);
   if (!ret) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret->calib = NULL;
   ret->relative_pose = NULL;

   error = rox_array2d_double_new(&ret->calib, 3, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&ret->relative_pose, 4, 4); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   rox_array2d_double_fillunit(ret->calib);
   rox_array2d_double_fillunit(ret->relative_pose);

   ret->scaler = 1.0;

   *obj = ret;

function_terminate:
   if (error) rox_bundle_camera_del(&ret);

   return error;
}

Rox_ErrorCode rox_bundle_camera_del(Rox_Bundle_Camera * obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Bundle_Camera todel = NULL;

   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *obj;
   *obj = NULL;

   if (!todel) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_array2d_double_del(&todel->calib);
   rox_array2d_double_del(&todel->relative_pose);

   rox_memory_delete(todel);

function_terminate:
   return error;
}
