//============================================================================
//
//    OPENROX   : File odometry_so3.h
//
//    Contents  : API of odometry_so3 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//============================================================================

#include "odometry_so3.h"

#include <baseproc/maths/maths_macros.h>

#include <baseproc/array/fill/fillunit.h>
#include <baseproc/array/norm/norm2sq.h>
#include <baseproc/maths/linalg/matso3.h>

#include <baseproc/array/crosscor/zncrosscor.h>
#include <baseproc/array/inverse/svdinverse.h>
#include <baseproc/array/multiply/mulmatmat.h>
#include <baseproc/array/add/add.h>
#include <baseproc/geometry/transforms/matsl3/sl3normalize.h>
#include <baseproc/geometry/transforms/transform_tools.h>
#include <baseproc/geometry/pixelgrid/warp_grid_matsl3.h>

#include <core/patch/patchplane.h>
#include <baseproc/calculus/linsys/linsys_texture_matso3_light_affine.h>
#include <core/templatesearch/region_zncc_search_mask_template_mask.h>

#include <inout/numeric/array2d_print.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_odometry_so3_new (Rox_Odometry_SO3 * obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Odometry_SO3 ret = NULL;


   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *obj = NULL;

   ret = (Rox_Odometry_SO3) rox_memory_allocate(sizeof(struct Rox_Odometry_SO3_Struct), 1);
   if (!ret) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   ret->homography = NULL;
   ret->Jtf = NULL;
   ret->JtJ = NULL;
   ret->bJtf = NULL;
   ret->bJtJ = NULL;
   ret->iJtJ = NULL;
   ret->solution = NULL;
   ret->solution_pose = NULL;
   ret->calibration_camera = NULL;
   ret->pose_so3 = NULL;
   ret->pose = NULL;
   ret->pose_right = NULL;


   error = rox_array2d_double_new(&ret->homography, 3, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&ret->pose_so3, 3, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&ret->pose, 4, 4); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&ret->pose_right, 4, 4); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&ret->calibration_camera, 3, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&ret->JtJ, 5, 5); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&ret->Jtf, 5, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&ret->bJtJ, 5, 5); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&ret->bJtf, 5, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&ret->iJtJ, 5, 5); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&ret->solution, 5, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new_subarray2d(&ret->solution_pose, ret->solution, 0, 0, 3, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillunit(ret->pose_so3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   *obj = ret;

function_terminate:
   if (error) rox_odometry_so3_del(&ret);

   return error;
}

Rox_ErrorCode rox_odometry_so3_del(Rox_Odometry_SO3 *obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Odometry_SO3 todel = NULL;


   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *obj;
   *obj = NULL;


   if (!todel) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_array2d_double_del(&todel->pose_so3);
   rox_array2d_double_del(&todel->pose_right);
   rox_array2d_double_del(&todel->pose);
   rox_array2d_double_del(&todel->calibration_camera);
   rox_array2d_double_del(&todel->homography);
   rox_array2d_double_del(&todel->JtJ);
   rox_array2d_double_del(&todel->Jtf);
   rox_array2d_double_del(&todel->bJtJ);
   rox_array2d_double_del(&todel->bJtf);
   rox_array2d_double_del(&todel->iJtJ);
   rox_array2d_double_del(&todel->solution_pose);
   rox_array2d_double_del(&todel->solution);

   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_odometry_so3_make (
   Rox_Odometry_SO3 obj,
   Rox_PatchPlane patch,
   Rox_Array2D_Float source,
   Rox_Sint max_iters
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double prev_zncc = 0.0;
   Rox_Double zncc = 0.0;


   if (!obj || !patch || !source) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double ** dsol = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dsol, obj->solution);

   error = rox_array2d_double_fillunit(obj->pose_so3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   prev_zncc = 0.0;
   for (Rox_Sint iter = 0; iter < max_iters; iter++)
   {

      error = rox_odometry_so3_get_pose(obj->pose, obj); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Prepare buffers
      error = rox_transformtools_build_homography ( obj->homography, obj->pose, obj->calibration_camera, obj->calibration_camera, 0, 0, 1, -1); 
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_patchplane_prepare_sl3(patch, obj->homography, source); 
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_patchplane_prepare_finish(patch); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      if (iter == 0)
      {
         error = rox_array2d_float_zncc(&prev_zncc, patch->reference, patch->warped_lum, patch->gradient_mask);
         ROX_ERROR_CHECK_TERMINATE ( error );
      }


      error = rox_jacobian_so3_simple_light_affine_premul(obj->JtJ, obj->Jtf, patch->gx, patch->gy, patch->mean, patch->difference, patch->gradient_mask, obj->calibration_camera); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Compute update to solution
      error = rox_array2d_double_svdinverse(obj->iJtJ, obj->JtJ); 
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_array2d_double_mulmatmat(obj->solution, obj->iJtJ, obj->Jtf); 
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_matso3_update_right(obj->pose_so3, obj->solution_pose); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      patch->alpha += (Rox_Float) dsol[3][0];
      patch->beta += (Rox_Float) dsol[4][0];
   }

   // Warp using last estimation

   error = rox_odometry_so3_get_pose(obj->pose, obj); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_transformtools_build_homography(obj->homography, obj->pose, obj->calibration_camera, obj->calibration_camera, 0, 0, -1, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_patchplane_prepare_sl3(patch, obj->homography, source); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_zncc(&zncc, patch->reference, patch->warped_lum, patch->gradient_mask);
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (zncc < prev_zncc)
   {
      error = ROX_ERROR_ALGORITHM_FAILURE;
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   return error;
}


Rox_ErrorCode rox_odometry_so3_make_stereo (
   Rox_Odometry_SO3 obj, 
   Rox_PatchPlane patch, 
   Rox_Array2D_Float source_left, 
   Rox_Array2D_Float source_right, 
   Rox_Array2D_Double pose_lr, 
   Rox_Sint max_iters
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!obj || !patch || !source_left || !source_right || !pose_lr) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double ** dsol = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dsol, obj->solution);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint iter = 0; iter < max_iters; iter++)
   {

      error = rox_odometry_so3_get_pose(obj->pose, obj); 
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_array2d_double_mulmatmat(obj->pose_right, pose_lr, obj->pose); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Prepare buffers for left image
      error = rox_transformtools_build_homography(obj->homography, obj->pose, obj->calibration_camera, obj->calibration_camera, 0, 0, -1, 1); 
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_patchplane_prepare_sl3(patch, obj->homography, source_left); 
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_patchplane_prepare_finish(patch); 
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_jacobian_so3_light_affine_premul(obj->JtJ, obj->Jtf, patch->gx, patch->gy, patch->mean, patch->difference, patch->gradient_mask, obj->pose, obj->calibration_camera); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Prepare buffers for right image
      error = rox_transformtools_build_homography(obj->homography, obj->pose_right, obj->calibration_camera, obj->calibration_camera, 0, 0, -1, 1); 
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_patchplane_prepare_sl3(patch, obj->homography, source_right); 
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_patchplane_prepare_finish(patch); 
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_jacobian_so3_light_affine_premul(obj->bJtJ, obj->bJtf, patch->gx, patch->gy, patch->mean, patch->difference, patch->gradient_mask, obj->pose_right, obj->calibration_camera); ROX_ERROR_CHECK_TERMINATE ( error );
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Compute update to solution
      error = rox_array2d_double_add(obj->JtJ, obj->JtJ, obj->bJtJ); 
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_array2d_double_add(obj->Jtf, obj->Jtf, obj->bJtf); 
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_array2d_double_svdinverse(obj->iJtJ, obj->JtJ); 
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_array2d_double_mulmatmat(obj->solution, obj->iJtJ, obj->Jtf); 
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_matso3_update_right(obj->pose_so3, obj->solution_pose); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      patch->alpha += (Rox_Float) dsol[3][0];
      patch->beta += (Rox_Float) dsol[4][0];
   }

   // Warp using last estimation

   error = rox_odometry_so3_get_pose(obj->pose, obj); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_transformtools_build_homography(obj->homography, obj->pose, obj->calibration_camera, obj->calibration_camera, 0, 0, -1, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_patchplane_prepare_sl3(patch, obj->homography, source_left); 
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_odometry_so3_get_pose(Rox_Array2D_Double pose, Rox_Odometry_SO3 obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;


   if (!obj || !pose) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_check_size(pose, 4, 4); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dto = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dto, pose);
   Rox_Double ** dti = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dti, obj->pose_so3);

   rox_array2d_double_fillunit(pose);

   dto[0][0] = dti[0][0];
   dto[0][1] = dti[0][1];
   dto[0][2] = dti[0][2];
   dto[1][0] = dti[1][0];
   dto[1][1] = dti[1][1];
   dto[1][2] = dti[1][2];
   dto[2][0] = dti[2][0];
   dto[2][1] = dti[2][1];
   dto[2][2] = dti[2][2];

function_terminate:
   return error;
}

Rox_ErrorCode rox_odometry_so3_set_calib(Rox_Odometry_SO3 obj, Rox_Array2D_Double calibration_camera)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!obj || !calibration_camera) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_copy(obj->calibration_camera, calibration_camera); 
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}
