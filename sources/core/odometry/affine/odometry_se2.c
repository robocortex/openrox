//============================================================================
//
//    OPENROX   : File odometry_se2.c
//
//    Contents  : Implementation of odometry_se2 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//============================================================================

#include "odometry_se2.h"

#include <baseproc/maths/maths_macros.h>

#include <baseproc/array/fill/fillunit.h>
#include <baseproc/array/norm/norm2sq.h>
#include <baseproc/maths/linalg/generators/algse2.h>
#include <baseproc/array/inverse/svdinverse.h>
#include <baseproc/array/multiply/mulmatmat.h>
#include <baseproc/geometry/transforms/matsl3/sl3normalize.h>
#include <baseproc/geometry/transforms/transform_tools.h>
#include <baseproc/geometry/pixelgrid/warp_grid_matsl3.h>

#include <core/patch/patchplane.h>
#include <baseproc/calculus/linsys/linsys_texture_matse2_light_affine_model2d.h>
#include <core/templatesearch/region_zncc_search_mask_template_mask.h>

#include <inout/numeric/array2d_print.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_odometry_se2_new(Rox_Odometry_SE2 *obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Odometry_SE2 ret = NULL;


   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *obj = NULL;

   ret = (Rox_Odometry_SE2) rox_memory_allocate(sizeof(struct Rox_Odometry_SE2_Struct), 1);

   if (!ret) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret->homography = NULL;
   ret->Jtf = NULL;
   ret->JtJ = NULL;
   ret->iJtJ = NULL;
   ret->solution = NULL;
   ret->solution_pose = NULL;
   ret->calibration_camera = NULL;
   ret->pose_se2 = NULL;
   ret->pose = NULL;


   error = rox_array2d_double_new(&ret->homography, 3, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&ret->pose_se2, 3, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&ret->pose, 4, 4); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&ret->calibration_camera, 3, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&ret->JtJ, 5, 5); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&ret->iJtJ, 5, 5); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&ret->Jtf, 5, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&ret->solution, 5, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new_subarray2d(&ret->solution_pose, ret->solution, 0, 0, 3, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   rox_array2d_double_fillunit(ret->pose_se2);

   *obj = ret;

function_terminate:
   if (error) rox_odometry_se2_del(&ret);

   return error;
}

Rox_ErrorCode rox_odometry_se2_del(Rox_Odometry_SE2 *obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Odometry_SE2 todel;

   if (!obj)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *obj;
   *obj = NULL;


   if (!todel) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_array2d_double_del(&todel->pose_se2);
   rox_array2d_double_del(&todel->pose);
   rox_array2d_double_del(&todel->calibration_camera);
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

Rox_ErrorCode rox_odometry_se2_make(Rox_Odometry_SE2 obj, Rox_PatchPlane patch, Rox_Array2D_Float source, Rox_Uint max_iters)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double norm = 0.0;


   if (!obj || !patch || !source) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double ** dsol = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dsol, obj->solution);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Uint iter = 0; iter < max_iters; iter++)
   {

      error = rox_odometry_se2_get_pose(obj->pose, obj); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Prepare buffers
      error = rox_transformtools_build_homography(obj->homography, obj->pose, obj->calibration_camera, obj->calibration_camera, 0, 0, -1, 1); 
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_patchplane_prepare_sl3(patch, obj->homography, source); 
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_patchplane_prepare_finish(patch); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_jacobian_se2_light_affine_premul(obj->JtJ, obj->Jtf, patch->gx, patch->gy, patch->mean, patch->difference, patch->gradient_mask, obj->pose_se2, obj->calibration_camera); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Compute update to solution
      error = rox_array2d_double_svdinverse(obj->iJtJ, obj->JtJ); 
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_array2d_double_mulmatmat(obj->solution, obj->iJtJ, obj->Jtf); 
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_linalg_se2update_right(obj->pose_se2, obj->solution_pose); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      patch->alpha += (Rox_Float) dsol[3][0];
      patch->beta += (Rox_Float) dsol[4][0];

      // Convergence test

      error = rox_array2d_double_norm2sq(&norm, obj->solution_pose); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      if (norm < 1e-8) break;
   }

   // Warp using last estimation

   error = rox_odometry_se2_get_pose(obj->pose, obj); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_transformtools_build_homography(obj->homography, obj->pose, obj->calibration_camera, obj->calibration_camera, 0, 0, -1, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_patchplane_prepare_sl3(patch, obj->homography, source); 
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_odometry_se2_get_pose(Rox_Array2D_Double pose, Rox_Odometry_SE2 obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;


   if (!obj || !pose) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_check_size(pose, 4, 4); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dto = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dto, pose);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dti = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dti, obj->pose_se2);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillunit(pose);
   ROX_ERROR_CHECK_TERMINATE ( error );

   dto[0][0] = dti[0][0];
   dto[0][1] = dti[0][1];
   dto[0][3] = dti[0][2];

   dto[1][0] = dti[1][0];
   dto[1][1] = dti[1][1];
   dto[1][3] = dti[1][2];

function_terminate:
   return error;
}

Rox_ErrorCode rox_odometry_se2_set_calib(Rox_Odometry_SE2 obj, Rox_Array2D_Double calibration_camera)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!obj || !calibration_camera)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_copy(obj->calibration_camera, calibration_camera); 
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}
