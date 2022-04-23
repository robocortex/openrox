//==============================================================================
//
//    OPENROX   : File edgepreproc_gray.c
//
//    Contents  : Implementation of edgepreproc_gray module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "edgepreproc_gray.h"

#include <system/errors/errors.h>
#include <system/memory/memory.h>

#include <baseproc/maths/kernels/gaussian2d.h>
#include <baseproc/image/convolve/array2d_uchar_symmetric_separable_convolve.h>
#include <baseproc/image/gradient/gradientsobel.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_edgepreproc_gray_new(Rox_EdgePreproc_Gray *obj, Rox_Sint width, Rox_Sint height)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_EdgePreproc_Gray ret = NULL;

   if (!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   *obj = NULL;

   ret = (Rox_EdgePreproc_Gray) rox_memory_allocate(sizeof(*ret), 1);
   if (!ret) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   // Init pointers to NULL
   ret->width = width;
   ret->height = height;
   ret->gradients = NULL;
   ret->filtered = NULL;
   ret->hkernel = NULL;
   ret->vkernel = NULL;

   error = rox_kernelgen_gaussian2d_separable_float_new(&ret->hkernel, &ret->vkernel, 1.0, 4.0);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_point2d_sshort_new(&ret->gradients, height, width);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // report creation in "rox_edgepreproc_gray_process" as we might not need it...
   // error = rox_array2d_uchar_new(&ret->filtered, height, width);

   *obj = ret;

function_terminate:
   if (error) rox_edgepreproc_gray_del(&ret);

   return error;
}

Rox_ErrorCode rox_edgepreproc_gray_del(Rox_EdgePreproc_Gray *obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_EdgePreproc_Gray todel = NULL;

   if (!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   todel = *obj;
   *obj = NULL;

   if (!todel) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   ROX_ERROR_CHECK(rox_array2d_point2d_sshort_del(&todel->gradients));
   ROX_ERROR_CHECK(rox_array2d_float_del(&todel->hkernel));
   ROX_ERROR_CHECK(rox_array2d_float_del(&todel->vkernel));
   // Test if filtered has been allocated
   if(todel->filtered) ROX_ERROR_CHECK(rox_array2d_uchar_del(&todel->filtered));

   rox_memory_delete(todel);

function_terminate:
    return error;
}

Rox_ErrorCode rox_edgepreproc_gray_process(Rox_EdgePreproc_Gray obj, Rox_Image source, Rox_Uint nbr_blur_passes)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!obj || !source) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   if(nbr_blur_passes > 0)
   {
      error = rox_array2d_uchar_new(&obj->filtered, obj->height, obj->width);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_uchar_symmetric_seperable_convolve(obj->filtered, source, obj->hkernel);
      ROX_ERROR_CHECK_TERMINATE ( error );

      for (Rox_Uint i = 1; i<nbr_blur_passes; ++i)
      {
         error = rox_array2d_uchar_symmetric_seperable_convolve(obj->filtered, obj->filtered, obj->hkernel);
         ROX_ERROR_CHECK_TERMINATE ( error );
      }

      error = rox_array2d_uchar_gradientsobel_nomask(obj->gradients, obj->filtered);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_uchar_del(&obj->filtered);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }
   else
   {
      error = rox_array2d_uchar_gradientsobel_nomask(obj->gradients, source);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   return error;
}
