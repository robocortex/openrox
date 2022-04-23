//==============================================================================
//
//    OPENROX   : File edgepreproc_rgb.c
//
//    Contents  : Implementation of edgepreproc_rgb module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "edgepreproc_rgb.h"

#include <system/errors/errors.h>
#include <system/memory/memory.h>

#include <baseproc/maths/kernels/gaussian2d.h>
#include <baseproc/image/convolve/array2d_uchar_symmetric_separable_convolve.h>
#include <baseproc/image/gradient/gradientsobel.h>
#include <baseproc/image/convert/roxrgba_split.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_edgepreproc_rgb_new(Rox_EdgePreproc_Rgb *obj, Rox_Sint width, Rox_Sint height)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_EdgePreproc_Rgb ret = NULL;

   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *obj = NULL;

   ret = (Rox_EdgePreproc_Rgb)rox_memory_allocate(sizeof(*ret), 1);
   if (!ret) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   ret->width = width;
   ret->height = height;
   ret->gradients = NULL;
   ret->filtered = NULL;
   ret->hkernel = NULL;
   ret->vkernel = NULL;
   ret->red = NULL;
   ret->green = NULL;
   ret->blue = NULL;
   ret->gradients_red = NULL;
   ret->gradients_green = NULL;
   ret->gradients_blue = NULL;

   error = rox_kernelgen_gaussian2d_separable_float_new(&ret->hkernel, &ret->vkernel, 1.0, 3.0);

   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_point2d_sshort_new(&ret->gradients, height, width);
   ROX_ERROR_CHECK_TERMINATE ( error );
    
   error = rox_array2d_point2d_sshort_new(&ret->gradients_red, height, width);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_point2d_sshort_new(&ret->gradients_green, height, width);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_point2d_sshort_new(&ret->gradients_blue, height, width);
   ROX_ERROR_CHECK_TERMINATE ( error );
    
   error = rox_array2d_uchar_new(&ret->filtered, height, width);
   ROX_ERROR_CHECK_TERMINATE ( error );
    
   error = rox_array2d_uchar_new(&ret->red, height, width);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_uchar_new(&ret->green, height, width);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uchar_new(&ret->blue, height, width);
   ROX_ERROR_CHECK_TERMINATE ( error );

   *obj = ret;

function_terminate:
   if (error) rox_edgepreproc_rgb_del(&ret);

   return error;
}

Rox_ErrorCode rox_edgepreproc_rgb_del(Rox_EdgePreproc_Rgb *obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_EdgePreproc_Rgb todel = NULL;

   if (!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   todel = *obj;
   *obj = NULL;

   if (!todel) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   rox_array2d_point2d_sshort_del(&todel->gradients);
   rox_array2d_point2d_sshort_del(&todel->gradients_red);
   rox_array2d_point2d_sshort_del(&todel->gradients_green);
   rox_array2d_point2d_sshort_del(&todel->gradients_blue);
   rox_array2d_uchar_del(&todel->filtered);
   rox_array2d_uchar_del(&todel->red);
   rox_array2d_uchar_del(&todel->green);
   rox_array2d_uchar_del(&todel->blue);
   rox_array2d_float_del(&todel->hkernel);
   rox_array2d_float_del(&todel->vkernel);

   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_edgepreproc_rgb_process(Rox_EdgePreproc_Rgb obj, Rox_Image_RGBA source)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;


   if (!obj || !source) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_roxrgba_split(obj->red, obj->green, obj->blue, source);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uchar_symmetric_seperable_convolve(obj->filtered, obj->red, obj->hkernel);

   ROX_ERROR_CHECK_TERMINATE ( error );
  
   error = rox_array2d_uchar_gradientsobel_nomask(obj->gradients_red, obj->filtered);
   ROX_ERROR_CHECK_TERMINATE ( error );
  
   error = rox_array2d_uchar_symmetric_seperable_convolve(obj->filtered, obj->green, obj->hkernel);
   ROX_ERROR_CHECK_TERMINATE ( error );
  
   error = rox_array2d_uchar_gradientsobel_nomask(obj->gradients_green, obj->filtered);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_uchar_symmetric_seperable_convolve(obj->filtered, obj->blue, obj->hkernel);
   ROX_ERROR_CHECK_TERMINATE ( error );
  
   error = rox_array2d_uchar_gradientsobel_nomask(obj->gradients_blue, obj->filtered);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Point2D_Sshort_Struct ** dd = NULL;
   error = rox_array2d_point2d_sshort_get_data_pointer_to_pointer(&dd, obj->gradients);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Point2D_Sshort_Struct ** dr = NULL;
   error = rox_array2d_point2d_sshort_get_data_pointer_to_pointer(&dr, obj->gradients_red);
   ROX_ERROR_CHECK_TERMINATE ( error );

   //Rox_Point2D_Sshort_Struct ** dg = rox_array2d_point2d_sshort_get_data(obj->gradients_green);
   //Rox_Point2D_Sshort_Struct ** db = rox_array2d_point2d_sshort_get_data(obj->gradients_blue);

   for ( Rox_Sint i = 0; i < obj->height; i++)
   {
      for ( Rox_Sint j = 0; j < obj->width; j++)
      {
         dd[i][j] = dr[i][j];
      }
   }

function_terminate:
   return error;
}
