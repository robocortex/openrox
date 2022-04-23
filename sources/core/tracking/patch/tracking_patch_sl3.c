//==============================================================================
//
//    OPENROX   : File tracking_patch_sl3.c
//
//    Contents  : Implementation of tracking_patch_sl3 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "tracking_patch_sl3.h"

#include <baseproc/maths/maths_macros.h>

#include <stdio.h>

#include <baseproc/array/norm/norm2sq.h>
#include <baseproc/maths/linalg/matsl3.h>
#include <baseproc/array/inverse/svdinverse.h>
#include <baseproc/array/multiply/mulmatmat.h>
#include <baseproc/geometry/transforms/matsl3/sl3normalize.h>
#include <baseproc/geometry/transforms/transform_tools.h>
#include <baseproc/geometry/pixelgrid/warp_grid_matsl3.h>

#include <core/patch/patchplane.h>
#include <baseproc/calculus/linsys/linsys_texture_matsl3_light_affine.h>
#include <core/templatesearch/region_zncc_search_mask_template_mask.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_tracking_patch_sl3_new(Rox_Tracking_Patch_SL3 * obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Tracking_Patch_SL3 ret = NULL;

   if (!obj)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *obj = NULL;

   ret = (Rox_Tracking_Patch_SL3)rox_memory_allocate(sizeof(struct Rox_Tracking_Patch_SL3_Struct), 1);

   if (!ret) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret->homography = NULL;
   ret->Jtf = NULL;
   ret->JtJ = NULL;
   ret->solution = NULL;
   ret->solution_pose = NULL;


   error = rox_array2d_double_new(&ret->homography, 3, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&ret->JtJ, 10, 10); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&ret->iJtJ, 10, 10); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&ret->Jtf, 10, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&ret->solution, 10, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new_subarray2d(&ret->solution_pose, ret->solution, 0, 0, 8, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   *obj = ret;

function_terminate:
   if(error) rox_tracking_patch_sl3_del(&ret);
   return error;
}

Rox_ErrorCode rox_tracking_patch_sl3_del(Rox_Tracking_Patch_SL3 * obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Tracking_Patch_SL3 todel = NULL;


   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *obj;
   *obj = NULL;


   if (!todel) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_array2d_double_del(&todel->homography);
   rox_array2d_double_del(&todel->JtJ);
   rox_array2d_double_del(&todel->iJtJ);
   rox_array2d_double_del(&todel->Jtf);
   rox_array2d_double_del(&todel->solution_pose);
   rox_array2d_double_del(&todel->solution);

   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_tracking_patch_sl3_make (
   Rox_Tracking_Patch_SL3 obj, 
   Rox_PatchPlane patch, 
   Rox_Array2D_Float source, 
   Rox_Uint max_iters)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double ** dsol = NULL; 
   error = rox_array2d_double_get_data_pointer_to_pointer( &dsol, obj->solution);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for ( Rox_Uint iter = 0; iter < max_iters; iter++)
   {
      // Prepare buffers

      error = rox_patchplane_prepare_sl3(patch, obj->homography, source); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_patchplane_prepare_finish(patch);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = linsys_texture_matsl3_light_affine(obj->JtJ, obj->Jtf, patch->mean, patch->difference, patch->gx, patch->gy, patch->gradient_mask);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Compute update to solution
      error = rox_array2d_double_svdinverse(obj->iJtJ, obj->JtJ);

      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_array2d_double_mulmatmat(obj->solution, obj->iJtJ, obj->Jtf);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_matsl3_update_right(obj->homography, obj->solution_pose);
      ROX_ERROR_CHECK_TERMINATE ( error );

      patch->alpha += (Rox_Float) dsol[8][0];
      patch->beta += (Rox_Float) dsol[9][0];

      // Convergence test
      Rox_Double norm = 0.0;
      error = rox_array2d_double_norm2sq(&norm, obj->solution_pose);
      ROX_ERROR_CHECK_TERMINATE ( error );
      if (norm < 1e-8) break;
   }

   // Reproject homography on SL(3)
   error = rox_matsl3_normalize(obj->homography, obj->homography);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Warp using last estimation
   error = rox_patchplane_prepare_sl3(patch, obj->homography, source);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_tracking_patch_sl3_set_homography(Rox_Tracking_Patch_SL3 obj, Rox_Array2D_Double homography)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   
   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_copy(obj->homography, homography);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_tracking_patch_sl3_get_homography(Rox_Array2D_Double homography, Rox_Tracking_Patch_SL3 obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!obj)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_copy(homography, obj->homography);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}
