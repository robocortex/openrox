//==============================================================================
//
//    OPENROX   : File tracking_patch_tu_tv_s_r.c
//
//    Contents  : Implementation of tracking_patch_tu_tv_s_r module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "tracking_patch_tu_tv_s_r.h"

#include <baseproc/array/norm/norm2sq.h>
#include <baseproc/maths/linalg/generators/algtutvsr.h>
#include <baseproc/calculus/linsys/linsys_texture_tutvsr_light_affine_model2d.h>
#include <baseproc/array/inverse/svdinverse.h>
#include <baseproc/array/multiply/mulmatmat.h>
#include <baseproc/geometry/transforms/matsl3/sl3normalize.h>
#include <core/patch/patchplane.h>
#include <baseproc/geometry/transforms/transform_tools.h>
#include <baseproc/geometry/pixelgrid/warp_grid_matsl3.h>
#include <core/templatesearch/region_zncc_search_mask_template_mask.h>
#include <inout/system/errors_print.h>

#include <baseproc/maths/maths_macros.h>

Rox_ErrorCode rox_tracking_patch_tu_tv_s_r_new(Rox_Tracking_Patch_tu_tv_s_r * obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Tracking_Patch_tu_tv_s_r ret = NULL;


   if (!obj)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *obj = NULL;

   ret = (Rox_Tracking_Patch_tu_tv_s_r) rox_memory_allocate(sizeof(*ret), 1);

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
   ret->s = 1.0;
   ret->r = 0.0;

   *obj = ret;

function_terminate:
   if (error) rox_tracking_patch_tu_tv_s_r_del(&ret);
   return error;
}

Rox_ErrorCode rox_tracking_patch_tu_tv_s_r_del(Rox_Tracking_Patch_tu_tv_s_r * obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Tracking_Patch_tu_tv_s_r todel = NULL;


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

Rox_ErrorCode rox_tracking_patch_tu_tv_s_r_make(Rox_Tracking_Patch_tu_tv_s_r obj, Rox_PatchPlane patch, Rox_Array2D_Float source, Rox_Uint max_iters)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;


   if (!obj)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double ** dsol = NULL; rox_array2d_double_get_data_pointer_to_pointer( &dsol, obj->solution);

   for ( Rox_Uint iter = 0; iter < max_iters; iter++)
   {
      error = rox_tracking_patch_tu_tv_s_r_to_SL3(obj->homography, obj->tu, obj->tv, obj->s, obj->r);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Prepare buffers

      error = rox_patchplane_prepare_sl3(patch, obj->homography, source);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_patchplane_prepare_finish(patch);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_jacobian_tutvsr_light_affine_premul(obj->JtJ, obj->Jtf, patch->mean, patch->difference, patch->gx, patch->gy, patch->gradient_mask);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Compute update to solution
      error = rox_array2d_double_svdinverse(obj->iJtJ, obj->JtJ);

      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_mulmatmat(obj->solution, obj->iJtJ, obj->Jtf);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_linalg_tutvsrupdate_right(obj->homography, obj->solution_pose);
      ROX_ERROR_CHECK_TERMINATE ( error );

      patch->alpha += (Rox_Float) dsol[4][0];
      patch->beta += (Rox_Float) dsol[5][0];

      error = rox_tracking_patch_tu_tv_s_r_from_SL3(&obj->tu, &obj->tv, &obj->s, &obj->r, obj->homography);
      ROX_ERROR_CHECK_TERMINATE ( error );

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

Rox_ErrorCode rox_tracking_patch_tu_tv_s_r_set_state(Rox_Tracking_Patch_tu_tv_s_r obj, Rox_Double tu, Rox_Double tv, Rox_Double s, Rox_Double r)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!obj)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   obj->tu = tu;
   obj->tv = tv;
   obj->s = s;
   obj->r = r;

function_terminate:
   return error;
}

Rox_ErrorCode rox_tracking_patch_tu_tv_s_r_get_state(Rox_Double * tu, Rox_Double *tv, Rox_Double *s, Rox_Double *r, Rox_Tracking_Patch_tu_tv_s_r obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;


   if (!obj)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!tu || !tv || !s || !r)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *tu = obj->tu;
   *tv = obj->tv;
   *s = obj->s;
   *r = obj->r;

function_terminate:
   return error;
}

Rox_ErrorCode rox_tracking_patch_tu_tv_s_r_to_SL3(Rox_Array2D_Double H, Rox_Double tu, Rox_Double tv, Rox_Double s, Rox_Double r)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;


   if (!H)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_check_size(H, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dh = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dh, H);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double cr = cos(r);
   Rox_Double sr = sin(r);

   dh[0][0] = cr * s;
   dh[0][1] = -sr * s;
   dh[0][2] = cr * tu - sr * tv;
   dh[1][0] = sr * s;
   dh[1][1] = cr * s;
   dh[1][2] = sr * tu + cr * tv;
   dh[2][0] = 0;
   dh[2][1] = 0;
   dh[2][2] = 1.0;

   Rox_Double det = s*s;
   Rox_Double normer = 1.0 / pow(det, 1.0/3.0);

   dh[0][0] *= normer; dh[0][1] *= normer; dh[0][2] *= normer;
   dh[1][0] *= normer; dh[1][1] *= normer; dh[1][2] *= normer;
   dh[2][0] *= normer; dh[2][1] *= normer; dh[2][2] *= normer;

function_terminate:
   return error;
}

Rox_ErrorCode rox_tracking_patch_tu_tv_s_r_from_SL3(Rox_Double *res_tu, Rox_Double *res_tv, Rox_Double *res_s, Rox_Double *res_r, Rox_Array2D_Double H)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double Hl[2][3];

   if (!H || !res_tu || !res_tv || !res_s || !res_r)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_check_size(H, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dh = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dh, H);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for ( Rox_Sint i = 0; i < 2; i++)
   {
      for ( Rox_Sint j = 0; j < 3; j++)
      {
         Hl[i][j] = dh[i][j] / dh[2][2];
      }
   }

   Rox_Double s = sqrt(Hl[0][0]*Hl[0][0] + Hl[1][0]*Hl[1][0]);
   Rox_Double cr = Hl[0][0] / s;
   Rox_Double sr = Hl[1][0] / s;
   Rox_Double r = atan2(sr, cr);

   Rox_Double denom = sr*sr + cr*cr;
   Rox_Double tu = (cr * Hl[0][2] + sr * Hl[1][2]) / denom;
   Rox_Double tv = (-sr * Hl[0][2] + cr * Hl[1][2]) / denom;

   *res_tu = tu;
   *res_tv = tv;
   *res_s = s;
   *res_r = r;

function_terminate:
   return error;
}
