//==============================================================================
//
//    OPENROX   : File tracking_patch_tu_tv_su_sv.c
//
//    Contents  : Implementation of tracking_patch_tu_tv_su_sv module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "tracking_patch_tu_tv_su_sv.h"

#include <baseproc/maths/maths_macros.h>
#include <baseproc/array/norm/norm2sq.h>
#include <baseproc/maths/linalg/generators/algtutvsusv.h>
#include <baseproc/array/inverse/svdinverse.h>
#include <baseproc/array/multiply/mulmatmat.h>
#include <baseproc/geometry/transforms/matsl3/sl3normalize.h>
#include <baseproc/geometry/transforms/transform_tools.h>
#include <baseproc/geometry/pixelgrid/warp_grid_matsl3.h>

#include <core/patch/patchplane.h>
#include <baseproc/calculus/linsys/linsys_texture_tutvsusv_light_affine_model2d.h>
#include <core/templatesearch/region_zncc_search_mask_template_mask.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_tracking_patch_tu_tv_su_sv_new(Rox_Tracking_Patch_tu_tv_su_sv * obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Tracking_Patch_tu_tv_su_sv ret = NULL;


   if (!obj)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *obj = NULL;

   ret = (Rox_Tracking_Patch_tu_tv_su_sv) rox_memory_allocate(sizeof(struct Rox_Tracking_Patch_tu_tv_su_sv_Struct), 1);

   if (!ret)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret->homography = NULL;
   ret->Jtf = NULL;
   ret->JtJ = NULL;
   ret->solution = NULL;
   ret->solution_pose = NULL;


   error = rox_array2d_double_new(&ret->homography, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&ret->JtJ, 6, 6);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&ret->iJtJ, 6, 6);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&ret->Jtf, 6, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&ret->solution, 6, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new_subarray2d(&ret->solution_pose, ret->solution, 0, 0, 4, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   ret->tu = 0.0;
   ret->tv = 0.0;
   ret->su = 1.0;
   ret->sv = 1.0;

   *obj = ret;

function_terminate:
   if (error) rox_tracking_patch_tu_tv_su_sv_del(&ret);
   return error;
}

Rox_ErrorCode rox_tracking_patch_tu_tv_su_sv_del(Rox_Tracking_Patch_tu_tv_su_sv * obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Tracking_Patch_tu_tv_su_sv todel = NULL;

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

Rox_ErrorCode rox_tracking_patch_tu_tv_su_sv_make(Rox_Tracking_Patch_tu_tv_su_sv obj, Rox_PatchPlane patch, Rox_Array2D_Float source, Rox_Uint max_iters)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double norm = 0.0;


   if (!obj)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double ** dsol = NULL; rox_array2d_double_get_data_pointer_to_pointer( &dsol, obj->solution);

   for ( Rox_Uint iter = 0; iter < max_iters; iter++)
   {
      rox_tracking_patch_tu_tv_su_sv_to_SL3(obj->homography, obj->tu, obj->tv, obj->su, obj->sv);
      // Prepare buffers

      error = rox_patchplane_prepare_sl3(patch, obj->homography, source);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_patchplane_prepare_finish(patch);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_jacobian_tutvsusv_light_affine_premul(obj->JtJ, obj->Jtf, patch->mean, patch->difference, patch->gx, patch->gy, patch->gradient_mask);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Compute update to solution
      error = rox_array2d_double_svdinverse(obj->iJtJ, obj->JtJ);

      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_mulmatmat(obj->solution, obj->iJtJ, obj->Jtf);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_linalg_tutvsusvupdate_right(obj->homography, obj->solution_pose);
      ROX_ERROR_CHECK_TERMINATE ( error );
      patch->alpha += (Rox_Float) dsol[4][0];
      patch->beta += (Rox_Float) dsol[5][0];

      rox_tracking_patch_tu_tv_su_sv_from_SL3(&obj->tu, &obj->tv, &obj->su, &obj->sv, obj->homography);

      // Convergence test
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

Rox_ErrorCode rox_tracking_patch_tu_tv_su_sv_set_state(Rox_Tracking_Patch_tu_tv_su_sv obj, Rox_Double tu, Rox_Double tv, Rox_Double su, Rox_Double sv)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   obj->tu = tu;
   obj->tv = tv;
   obj->su = su;
   obj->sv = sv;

function_terminate:
   return error;
}

Rox_ErrorCode rox_tracking_patch_tu_tv_su_sv_get_state(Rox_Double * tu, Rox_Double *tv, Rox_Double *su, Rox_Double *sv, Rox_Tracking_Patch_tu_tv_su_sv obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   if (!tu || !tv || !su || !sv) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   *tu = obj->tu;
   *tv = obj->tv;
   *su = obj->su;
   *sv = obj->sv;

function_terminate:
   return error;
}

Rox_ErrorCode rox_tracking_patch_tu_tv_su_sv_to_SL3(Rox_Array2D_Double H, Rox_Double tu, Rox_Double tv, Rox_Double su, Rox_Double sv)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;


   if ( !H )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_check_size(H, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dh = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dh, H);

   dh[0][0] = su;
   dh[0][1] = 0.0;
   dh[0][2] = tu;

   dh[1][0] = 0.0;
   dh[1][1] = sv;
   dh[1][2] = tv;

   dh[2][0] = 0.0;
   dh[2][1] = 0.0;
   dh[2][2] = 1.0;

function_terminate:
   return error;
}

Rox_ErrorCode rox_tracking_patch_tu_tv_su_sv_from_SL3(Rox_Double *res_tu, Rox_Double *res_tv, Rox_Double *res_su, Rox_Double *res_sv, Rox_Array2D_Double H)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double su = 1.0, sv = 1.0, tu = 0.0, tv = 0.0;

   if (!H) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   if (!res_tu || !res_tv || !res_su || !res_sv) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   error = rox_array2d_double_check_size(H, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dh = NULL;
   rox_array2d_double_get_data_pointer_to_pointer( &dh, H);

   if(dh[2][2] > 0.0)
   {
      su = dh[0][0] / dh[2][2];
      sv = dh[1][1] / dh[2][2];
      tu = dh[0][2] / dh[2][2];
      tv = dh[1][2] / dh[2][2];
   }

   *res_tu = tu;
   *res_tv = tv;
   *res_su = su;
   *res_sv = sv;

function_terminate:
   return error;
}
