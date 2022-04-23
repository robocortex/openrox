//==============================================================================
//
//    OPENROX   : File odometry_plane.c
//
//    Contents  : Implementation of odometry_plane module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "odometry_plane.h"
#include "odometry_plane_struct.h"

#include <baseproc/maths/maths_macros.h>

#include <baseproc/array/norm/norm2sq.h>
#include <baseproc/maths/linalg/matse3.h>
#include <baseproc/array/inverse/svdinverse.h>
#include <baseproc/array/multiply/mulmatmat.h>
#include <baseproc/geometry/transforms/matsl3/sl3normalize.h>
#include <baseproc/geometry/transforms/transform_tools.h>
#include <baseproc/geometry/pixelgrid/warp_grid_matsl3.h>

#include <baseproc/calculus/linsys/linsys_se3_light_affine_premul.h>
#include <baseproc/calculus/linsys/linsys_se3_z1_light_affine_premul.h>

#include <baseproc/calculus/linsys/linsys_texture_matse3_light_affine_model2d.h>

#include <core/templatesearch/region_zncc_search_mask_template_mask.h>
#include <core/patch/patchplane.h>

#include <inout/system/print.h>
#include <inout/system/errors_print.h>
#include <inout/numeric/array2d_print.h>

Rox_ErrorCode rox_odometry_plane_new (
   Rox_Odometry_Plane *odometry_plane
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Odometry_Plane ret = NULL;

   if (!odometry_plane) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *odometry_plane = NULL;

   ret = (Rox_Odometry_Plane) rox_memory_allocate(sizeof(struct Rox_Odometry_Plane_Struct), 1);
   if (!ret) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret->homography           = NULL;
   ret->Jtf                  = NULL;
   ret->JtJ                  = NULL;
   ret->iJtJ                 = NULL;
   ret->solution             = NULL;
   ret->solution_pose        = NULL;
   ret->calibration_camera   = NULL;
   ret->calibration_template = NULL;
   ret->pose                 = NULL;

   error = rox_matsl3_new ( &ret->homography );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matse3_new ( &ret->pose );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matut3_new ( &ret->calibration_camera );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matut3_new ( &ret->calibration_template );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matrix_new ( &ret->JtJ, 8, 8 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matrix_new ( &ret->iJtJ, 8, 8);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matrix_new ( &ret->Jtf, 8, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matrix_new ( &ret->solution, 8, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new_subarray2d(&ret->solution_pose, ret->solution, 0, 0, 6, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   *odometry_plane = ret;

function_terminate:
   if (error) rox_odometry_plane_del(&ret);

   return error;
}


Rox_ErrorCode rox_odometry_plane_del (
   Rox_Odometry_Plane *odometry_plane
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Odometry_Plane todel;


   if (!odometry_plane) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *odometry_plane;
   *odometry_plane = NULL;


   if (!todel) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_matse3_del(&todel->pose);
   rox_matut3_del(&todel->calibration_camera);
   rox_matut3_del(&todel->calibration_template);
   rox_matsl3_del(&todel->homography);
   rox_matrix_del(&todel->JtJ);
   rox_matrix_del(&todel->iJtJ);
   rox_matrix_del(&todel->Jtf);
   rox_matrix_del(&todel->solution);
   rox_array2d_double_del(&todel->solution_pose);

   rox_memory_delete(todel);

function_terminate:
   return error;
}

// #define POSE_SHIFT_TZ1 0 // NEW STYLE
#define POSE_SHIFT_TZ1 1 // OLD STYLE OK

Rox_ErrorCode rox_odometry_plane_make (
   Rox_Odometry_Plane odometry_plane,
   Rox_PatchPlane patch,
   Rox_Array2D_Float source,
   Rox_Sint max_iters
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double norm = 0.0;


   if ( !odometry_plane || !patch || !source ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double ** dsol = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dsol, odometry_plane->solution);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for ( Rox_Sint iter = 0; iter < max_iters; iter++ )
   {
      // Build homography from pose
      if (POSE_SHIFT_TZ1 == 1)
      {
         error = rox_transformtools_build_homography(odometry_plane->homography, odometry_plane->pose, odometry_plane->calibration_camera, odometry_plane->calibration_template, 0, 0, -1, 1);
         ROX_ERROR_CHECK_TERMINATE ( error );
      }
      else
      {
         // New code to replace rox_transformtools_build_homography function

         Rox_MatUT3 Kc    = odometry_plane->calibration_camera;
         Rox_MatSE3 c_T_o = odometry_plane->pose;
         Rox_MatSL3 t_G_o = odometry_plane->calibration_template;
         Rox_MatSL3 c_G_t = odometry_plane->homography;
         Rox_MatSL3 c_G_o = NULL; // should be permanently stored in the odometry object ?

         error = rox_matsl3_new ( &c_G_o ) ;
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_transformtools_build_model_to_image_homography ( c_G_o, Kc, c_T_o );
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_matsl3_mulmatinv ( c_G_t, c_G_o, t_G_o );
         ROX_ERROR_CHECK_TERMINATE ( error );

         rox_matsl3_del ( &c_G_o );
      }

      // Prepare buffers
      error = rox_patchplane_prepare_sl3 ( patch, odometry_plane->homography, source );
      ROX_ERROR_CHECK_TERMINATE ( error );


      error = rox_patchplane_prepare_finish ( patch );
      ROX_ERROR_CHECK_TERMINATE ( error );
   
      if ( POSE_SHIFT_TZ1 == 1 )
      {
         error = rox_linsys_se3_z1_light_affine_premul ( odometry_plane->JtJ, odometry_plane->Jtf, patch->gx, patch->gy, patch->mean, patch->difference, patch->gradient_mask, odometry_plane->pose, odometry_plane->calibration_template);
         ROX_ERROR_CHECK_TERMINATE ( error );
      }
      else
      {
         // New code to replace rox_jacobian_se3_z1_light_affine_premul function
         error = rox_jacobian_se3_light_affine_premul ( odometry_plane->JtJ, odometry_plane->Jtf, patch->gx, patch->gy, patch->mean, patch->difference, patch->gradient_mask, odometry_plane->pose, odometry_plane->calibration_template);
         ROX_ERROR_CHECK_TERMINATE ( error );
      }

      // Compute update to solution
      error = rox_array2d_double_svdinverse ( odometry_plane->iJtJ, odometry_plane->JtJ );
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_mulmatmat ( odometry_plane->solution, odometry_plane->iJtJ, odometry_plane->Jtf );
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Update pose on the right : pose_new = pose_old
      error = rox_matse3_update_right ( odometry_plane->pose, odometry_plane->solution_pose );
      ROX_ERROR_CHECK_TERMINATE ( error );

      patch->alpha += (Rox_Float) dsol[6][0];
      patch->beta  += (Rox_Float) dsol[7][0];

      // Convergence test on global velocity with fixed threshold
      // should be split into two tests for translation and rotation velocity with tunable threshold
      error = rox_array2d_double_norm2sq ( &norm, odometry_plane->solution_pose );
      ROX_ERROR_CHECK_TERMINATE ( error );

      if (norm < 1e-8)
      {
         break;
      }
   }

   if(POSE_SHIFT_TZ1 == 1)
   {
      // Warp using last estimation
      error = rox_transformtools_build_homography ( odometry_plane->homography, odometry_plane->pose, odometry_plane->calibration_camera, odometry_plane->calibration_template, 0, 0, 1, -1);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }
   else
   {
      // New code to replace rox_transformtools_build_homography function

         Rox_MatUT3 Kc    = odometry_plane->calibration_camera;
         Rox_MatSE3 c_T_o = odometry_plane->pose;
         Rox_MatSL3 t_G_o = odometry_plane->calibration_template;
         Rox_MatSL3 c_G_t = odometry_plane->homography;
         Rox_MatSL3 c_G_o = NULL; // should be permanently stored in the odometry object ?

         error = rox_matsl3_new ( &c_G_o ) ;
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_transformtools_build_model_to_image_homography ( c_G_o, Kc, c_T_o );
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_matsl3_mulmatinv ( c_G_t, c_G_o, t_G_o );
         ROX_ERROR_CHECK_TERMINATE ( error );

         rox_matsl3_del ( &c_G_o );
      }

   error = rox_patchplane_prepare_sl3(patch, odometry_plane->homography, source);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}


Rox_ErrorCode rox_odometry_plane_set_pose (
   Rox_Odometry_Plane odometry_plane,
   const Rox_MatSE3 pose,
   const Rox_MatUT3 calibration_camera,
   const Rox_MatSL3 calibration_template
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !odometry_plane )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( !pose || !calibration_camera || !calibration_template )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_copy(odometry_plane->pose, pose);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_copy(odometry_plane->calibration_camera, calibration_camera);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_copy(odometry_plane->calibration_template, calibration_template);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}


Rox_ErrorCode rox_odometry_plane_get_pose (
   Rox_MatSE3 pose,
   const Rox_Odometry_Plane odometry_plane
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!odometry_plane || !pose) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_copy(pose, odometry_plane->pose);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}
