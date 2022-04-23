//==============================================================================
//
//    OPENROX   : File odometry_plane_robustlight.c
//
//    Contents  : Implementation of odometry_plane_robustlight module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "odometry_plane_robustlight.h"
#include "odometry_plane_robustlight_struct.h"

#include <baseproc/maths/maths_macros.h>
#include <float.h>

#include <baseproc/array/norm/norm2sq.h>
#include <baseproc/maths/linalg/matse3.h>
#include <baseproc/array/inverse/svdinverse.h>
#include <baseproc/array/multiply/mulmatmat.h>
#include <baseproc/array/multiply/mulmattransmat.h>
#include <baseproc/array/fill/fillval.h>
#include <baseproc/array/add/add.h>
#include <baseproc/array/substract/substract.h>
#include <baseproc/geometry/transforms/matsl3/sl3normalize.h>
#include <baseproc/geometry/transforms/transform_tools.h>
#include <baseproc/geometry/pixelgrid/warp_grid_matsl3.h>

#include <core/patch/patchplane.h>
#include <baseproc/calculus/linsys/linsys_se3_z1_light_affine_premul.h>

#include <baseproc/calculus/linsys/linsys_texture_matse3_light_affine_model2d.h>

#include <core/templatesearch/region_zncc_search_mask_template_mask.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_odometry_plane_robustlight_new(Rox_Odometry_Plane_RobustLight *obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Odometry_Plane_RobustLight ret = NULL;

   if (!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   *obj = NULL;

   ret = (Rox_Odometry_Plane_RobustLight) rox_memory_allocate(sizeof(struct Rox_Odometry_Plane_RobustLight_Struct), 1);
   if (!ret) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   ret->homography = NULL;
   ret->lJtf = NULL;
   ret->lJtJ = NULL;
   ret->calibration_camera = NULL;
   ret->calibration_template = NULL;
   ret->calibration_template_block = NULL;
   ret->pose = NULL;


   error = rox_array2d_double_new(&ret->homography, 3, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&ret->pose, 4, 4); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&ret->calibration_camera, 3, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&ret->calibration_template, 3, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&ret->calibration_template_block, 3, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&ret->lJtJ, 8, 8); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&ret->lJtf, 8, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   *obj = ret;
   error = ROX_ERROR_NONE;

function_terminate:
   if (error) rox_odometry_plane_robustlight_del(&ret);
   return error;
}

Rox_ErrorCode rox_odometry_plane_robustlight_del(Rox_Odometry_Plane_RobustLight *obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Odometry_Plane_RobustLight todel = NULL;

   if (!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   todel = *obj;
   *obj = NULL;

   if (!todel) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   rox_array2d_double_del(&todel->pose);
   rox_array2d_double_del(&todel->calibration_camera);
   rox_array2d_double_del(&todel->calibration_template);
   rox_array2d_double_del(&todel->calibration_template_block);
   rox_array2d_double_del(&todel->homography);
   rox_array2d_double_del(&todel->lJtJ);
   rox_array2d_double_del(&todel->lJtf);

   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_odometry_plane_robustlight_make(Rox_Odometry_Plane_RobustLight obj, Rox_PatchPlane_RobustLight patch, Rox_Array2D_Float source, Rox_Sint max_iters)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint iter;
   Rox_Uint idblock, nbblocks, l, c, count_iters_nonbetter;
   Rox_Double normtr, normrot, preverror;
   Rox_Double **djptf, **dljtf, **djptjp, **dljtj, **djatjp, **djatf, **dijatja, **dsolp, **dsola;
   Rox_Array2D_Double JptJp, Jptf, Jatf, JatJp, iJatJa;
   Rox_Array2D_Double DinvB, BpDinvB, Dinvya, BpDinvya, errp, solutiona, Mp, iMp, solutionp, solution_pose, Bxp, yamBxp;

   // A = JptJp
   // B = JatJp
   // D = JatJa
   // yp = Jptf
   // ya = Jatf
   //
   // [ A * xp + B' * xa = yp
   // [ B * xp + D  * xa = ya
   //
   // [ A * xp + B' * xa = yp
   // [ B'D^-1B * xp + B'D^-1D * xa = B'D^-1 * ya
   //
   // [ A * xp + B' * xa = yp
   // [ B'D^-1B * xp + B' * xa = B'D^-1 * ya
   //
   // [ A * xp + B' * xa = yp
   // [ B' * xa = B'D^-1 * ya - B'D^-1B * xp
   //
   // [ A * xp + B'D^-1 * ya - B'D^-1B * xp = yp
   // [ xa = D^-1 * ya - D^-1B * xp
   //
   // [ A * xp - B'D^-1B * xp = yp - B'D^-1 * ya
   // [ xa = D^-1 * ya - D^-1B * xp
   //
   // [ (A - B'D^-1B) * xp = yp - B'D^-1 * ya
   // [ xa = D^-1 * (ya - B * xp)

   if (!obj || !patch) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   nbblocks = patch->alphas->used;
   if (nbblocks == 0) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error)}

   JptJp = NULL;
   Jptf = NULL;
   JatJp = NULL;
   Jatf = NULL;
   iJatJa = NULL;
   DinvB = NULL;
   Dinvya = NULL;
   BpDinvya = NULL;
   errp = NULL;
   BpDinvB = NULL;
   Mp = NULL;
   iMp = NULL;
   solutionp = NULL;
   solution_pose = NULL;
   Bxp = NULL;
   yamBxp = NULL;
   solutiona = NULL;


   error = rox_array2d_double_new(&JptJp, 7, 7); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&Jptf, 7, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&JatJp, nbblocks, 7); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&Jatf, nbblocks, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&iJatJa, nbblocks, nbblocks); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&DinvB, nbblocks, 7); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&Dinvya, nbblocks, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&BpDinvya, 7, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&errp, 7, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&BpDinvB, 7, 7); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&Mp, 7, 7); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&iMp, 7, 7); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&solutionp, 7, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&Bxp, nbblocks, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&yamBxp, nbblocks, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&solutiona, nbblocks, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new_subarray2d(&solution_pose, solutionp, 0, 0, 6, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer_to_pointer( &djptf, Jptf);
   error = rox_array2d_double_get_data_pointer_to_pointer( &dljtf, obj->lJtf);
   error = rox_array2d_double_get_data_pointer_to_pointer( &djptjp, JptJp);
   error = rox_array2d_double_get_data_pointer_to_pointer( &dljtj, obj->lJtJ);
   error = rox_array2d_double_get_data_pointer_to_pointer( &djatjp, JatJp);
   error = rox_array2d_double_get_data_pointer_to_pointer( &djatf, Jatf);
   error = rox_array2d_double_get_data_pointer_to_pointer( &dijatja, iJatJa);
   error = rox_array2d_double_get_data_pointer_to_pointer( &dsola, solutiona);
   error = rox_array2d_double_get_data_pointer_to_pointer( &dsolp, solutionp);

   preverror = DBL_MAX;
   count_iters_nonbetter = 0;

   for (iter = 0; iter < max_iters; iter++)
   {
      // Prepare homography from pose
      error = rox_transformtools_build_homography(obj->homography, obj->pose, obj->calibration_camera, obj->calibration_template, 0, 0, -1, 1); ROX_ERROR_CHECK_TERMINATE(error)

      // Prepare buffers

      error = rox_patchplane_robustlight_prepare_sl3(patch, obj->homography, source); 
      ROX_ERROR_CHECK_TERMINATE ( error );
      error = rox_patchplane_robustlight_prepare_finish(patch); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      if (patch->error >= preverror)
      {
         count_iters_nonbetter++;
         if (count_iters_nonbetter >= 3)
         {
            break;
         }
      }
      preverror = patch->error;

      rox_array2d_double_fillval(JptJp, 0);
      rox_array2d_double_fillval(Jptf, 0);
      rox_array2d_double_fillval(JatJp, 0);
      rox_array2d_double_fillval(iJatJa, 0);
      rox_array2d_double_fillval(Jatf, 0);

      for (idblock = 0; idblock < patch->alphas->used; idblock++)
      {
         Rox_Double pixmean = patch->pixelmeans->data[idblock];

         // Check saturation
         if (pixmean < .04  || pixmean > .95)
         {
            djatf[idblock][0] = 0;
            dijatja[idblock][idblock] = 0;
         }

         // Take care of the shift of the block relative to the global patch
         rox_array2d_double_copy(obj->calibration_template_block, obj->calibration_template);
         rox_transformtools_homography_shiftleft(obj->calibration_template_block,  -patch->toplefts->data[idblock].u, -patch->toplefts->data[idblock].v);

         error = rox_linsys_se3_z1_light_affine_premul(obj->lJtJ, obj->lJtf, patch->subs_gx->data[idblock], patch->subs_gy->data[idblock], patch->subs_mean->data[idblock], patch->subs_difference->data[idblock], patch->subs_gradient_mask->data[idblock], obj->pose, obj->calibration_template_block);
         ROX_ERROR_CHECK_TERMINATE(error)

         // Copy pose JtJ
         for (l = 0; l < 6; l++)
         {
            for (c = 0; c < 6; c++)
            {
               djptjp[l][c] += dljtj[l][c];
            }

            djptf[l][0] += dljtf[l][0];
         }

         // Copy beta related values
         djptf[6][0] += dljtf[7][0];
         for (l = 0; l < 6; l++)
         {
            djptjp[6][l] += dljtj[7][l];
         }
         djptjp[6][6] += dljtj[7][7];

         // Copy alpha related values
         if (fabs(dljtj[6][6]) < DBL_EPSILON) dijatja[idblock][idblock] = 0;
         else dijatja[idblock][idblock] = 1.0 / dljtj[6][6];

         djatf[idblock][0] = dljtf[6][0];
         for (l = 0; l < 6; l++)
         {
            djatjp[idblock][l] = dljtj[6][l];
         }
         djatjp[idblock][6] = dljtj[6][7];
      }

      // Compute update to solution

      error = rox_array2d_double_mulmatmat(DinvB, iJatJa, JatJp); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_mulmatmat(Dinvya, iJatJa, Jatf); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_mulmattransmat(BpDinvya, JatJp, Dinvya); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_substract(errp, Jptf, BpDinvya); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_mulmattransmat(BpDinvB, JatJp, DinvB); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_substract(Mp, JptJp, BpDinvB); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_svdinverse(iMp, Mp);  
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_mulmatmat(solutionp, iMp, errp);  
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Compute update for alphas
      error = rox_array2d_double_mulmatmat(Bxp, JatJp, solutionp); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_substract(yamBxp, Jatf, Bxp); 
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_array2d_double_mulmatmat(solutiona, iJatJa, yamBxp); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_matse3_update_right(obj->pose, solution_pose); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      patch->beta += (Rox_Float) dsolp[6][0];

      for (idblock = 0; idblock < patch->alphas->used; idblock++)
      {
         patch->alphas->data[idblock] += dsola[idblock][0];
      }

      // Convergence test
      normtr = sqrt(dsolp[0][0] * dsolp[0][0] + dsolp[1][0] * dsolp[1][0] + dsolp[2][0] * dsolp[2][0]);
      normrot = sqrt(dsolp[3][0] * dsolp[3][0] + dsolp[4][0] * dsolp[4][0] + dsolp[5][0] * dsolp[5][0]);
      if (normtr < 1e-3 && normrot < 1e-3) break;
   }

   // Warp using last estimation

   error = rox_transformtools_build_homography(obj->homography, obj->pose, obj->calibration_camera, obj->calibration_template, 0, 0, 1, -1); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_patchplane_robustlight_prepare_sl3(patch, obj->homography, source); 
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   rox_array2d_double_del(&JptJp);
   rox_array2d_double_del(&Jptf);
   rox_array2d_double_del(&JatJp);
   rox_array2d_double_del(&Jatf);
   rox_array2d_double_del(&iJatJa);
   rox_array2d_double_del(&DinvB);
   rox_array2d_double_del(&Dinvya);
   rox_array2d_double_del(&BpDinvya);
   rox_array2d_double_del(&errp);
   rox_array2d_double_del(&BpDinvB);
   rox_array2d_double_del(&Mp);
   rox_array2d_double_del(&iMp);
   rox_array2d_double_del(&solutionp);
   rox_array2d_double_del(&solution_pose);
   rox_array2d_double_del(&Bxp);
   rox_array2d_double_del(&yamBxp);
   rox_array2d_double_del(&solutiona);

   return error;
}

Rox_ErrorCode rox_odometry_plane_robustlight_set_pose(Rox_Odometry_Plane_RobustLight obj, Rox_Array2D_Double pose, Rox_Array2D_Double calibration_camera, Rox_Array2D_Double calibration_template)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}


   error = rox_array2d_double_copy(obj->pose, pose); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_copy(obj->calibration_camera, calibration_camera); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_copy(obj->calibration_template, calibration_template); 
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_odometry_plane_robustlight_get_pose(Rox_Array2D_Double pose, Rox_Odometry_Plane_RobustLight obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   error = rox_array2d_double_copy(pose, obj->pose);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}
