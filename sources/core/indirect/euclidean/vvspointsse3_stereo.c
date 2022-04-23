//==============================================================================
//
//    OPENROX   : File vvspointsse3_stereo.c
//
//    Contents  : Implementation of vvspointsse3_stereo module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "vvspointsse3_stereo.h"
#include "vvspointsse3.h"

#include <baseproc/maths/maths_macros.h>
#include <float.h>
#include <string.h>

#include <generated/dynvec_point2d_float_struct.h>
#include <generated/dynvec_point3d_float_struct.h>
#include <generated/dynvec_sint_struct.h>
#include <generated/dynvec_double_struct.h>

#include <baseproc/geometry/point/point3d_matse3_transform.h>
#include <baseproc/geometry/point/point2d_projection_from_point3d.h>
#include <baseproc/geometry/point/dynvec_point2d_tools.h>
#include <baseproc/geometry/transforms/transform_tools.h>
#include <baseproc/array/multiply/mulmatmat.h>
#include <baseproc/array/inverse/svdinverse.h>
#include <baseproc/array/add/add.h>
#include <baseproc/array/robust/tukey.h>
#include <baseproc/maths/linalg/matse3.h>
#include <baseproc/array/fill/fillval.h>
#include <baseproc/array/fill/fillunit.h>

#include <baseproc/calculus/linsys/linsys_point2d_pix_matse3_weighted.h>
#include <baseproc/calculus/linsys/linsys_point2d_nor_matse3_weighted.h>
#include <baseproc/calculus/linsys/linsys_stereo_point2d_pix_matse3_weighted.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_jacobian_se3_flat_solver(Rox_Array2D_Double pose, Rox_Array2D_Double JtJ, Rox_Array2D_Double Jtf);

Rox_ErrorCode rox_jacobian_se3_stereo_solver(Rox_Array2D_Double pose, Rox_Array2D_Double JtJ, Rox_Array2D_Double Jtf)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double sol = NULL, iJtJ = NULL;

   if (!JtJ || !Jtf || !pose) 
   {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   error = rox_array2d_double_check_size(pose, 4, 4); ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_check_size(JtJ, 6, 6); ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_check_size(Jtf, 6, 1); ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&sol, 6, 1); ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_new(&iJtJ, 6, 6); ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_svdinverse(iJtJ, JtJ); ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_mulmatmat(sol, iJtJ, Jtf); ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_matse3_update_right(pose, sol); ROX_ERROR_CHECK_TERMINATE(error)

function_terminate:
   rox_array2d_double_del(&iJtJ);
   rox_array2d_double_del(&sol);

   return error;
}

Rox_ErrorCode rox_points_float_refine_pose_vvs_stereo(Rox_Array2D_Double pose, Rox_Array2D_Double calib_left, Rox_Array2D_Double calib_right, Rox_Array2D_Double rTl, Rox_DynVec_Point2D_Float measures_left, Rox_DynVec_Point2D_Float measures_right, Rox_DynVec_Point3D_Float references, Rox_DynVec_Sint levels, Rox_DynVec_Double weights, Rox_Double maxdist_prefilter)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint nb_points, idpt, iter;
   Rox_Array2D_Double pose_right, diff, dist1, dist2, JtJ, Jtf, JtJ_left, JtJ_right, Jtf_left, Jtf_right, eye44;
   Rox_Array2D_Double weight;
   Rox_Array2D_Double work1;
   Rox_Array2D_Double work2;
   Rox_Double lsqsum, sqsum1, sqsum2, delta, invcount, maxdist;
   Rox_Double * dw = NULL;

   const Rox_Uint max_iter = 100;

   Rox_DynVec_Point3D_Float mreft_left, mreft_right;
   Rox_DynVec_Point2D_Float mrefp_left, mrefp_right;
   Rox_DynVec_Point3D_Float mreft_left_filtered, mreft_right_filtered;
   Rox_DynVec_Point2D_Float mrefp_left_filtered, mrefp_right_filtered;
   Rox_DynVec_Point2D_Float measures_left_filtered, measures_right_filtered;
   Rox_DynVec_Point3D_Float references_filtered;
   Rox_DynVec_Double weights_filtered;

   if (!pose || !rTl || !calib_left || !calib_right || !measures_left || !measures_right || !references || !levels || !weights) return ROX_ERROR_NULL_POINTER;

   nb_points = measures_left->used;

   if (nb_points < 4) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error)}
   if (references->used != nb_points) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error)}

   mreft_left = NULL;
   mrefp_left = NULL;
   mreft_right = NULL;
   mrefp_right = NULL;
   mreft_left_filtered = NULL;
   mrefp_left_filtered = NULL;
   mreft_right_filtered = NULL;
   mrefp_right_filtered = NULL;
   measures_left_filtered = NULL;
   measures_right_filtered = NULL;
   references_filtered = NULL;
   weights_filtered = NULL;
   diff = NULL;
   dist1 = NULL;
   dist2 = NULL;
   work1 = NULL;
   work2 = NULL;
   weight = NULL;
   JtJ = NULL;
   Jtf = NULL;
   JtJ_left = NULL;
   JtJ_right = NULL;
   Jtf_left = NULL;
   Jtf_right = NULL;
   pose_right = NULL;
   eye44 = NULL;

   error = rox_array2d_double_new(&eye44, 4, 4); ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_new(&pose_right, 4, 4); ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_new(&JtJ, 6, 6); ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_new(&Jtf, 6, 1); ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_new(&JtJ_left, 6, 6); ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_new(&Jtf_left, 6, 1); ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_new(&JtJ_right, 6, 6); ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_new(&Jtf_right, 6, 1); ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_dynvec_point3d_float_new(&references_filtered, 100); ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_dynvec_point3d_float_new(&mreft_left, 100); ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_dynvec_point2d_float_new(&mrefp_left, 100); ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_dynvec_point3d_float_new(&mreft_right, 100); ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_dynvec_point2d_float_new(&mrefp_right, 100); ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_dynvec_point3d_float_new(&mreft_left_filtered, 100); ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_dynvec_point2d_float_new(&mrefp_left_filtered, 100); ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_dynvec_point3d_float_new(&mreft_right_filtered, 100); ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_dynvec_point2d_float_new(&mrefp_right_filtered, 100); ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_dynvec_double_new(&weights_filtered, 100); ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_dynvec_point2d_float_new(&measures_left_filtered, 100); ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_dynvec_point2d_float_new(&measures_right_filtered, 100); ROX_ERROR_CHECK_TERMINATE(error)

   // Allocate dynamic
   rox_dynvec_point3d_float_reset(mreft_left);
   rox_dynvec_point3d_float_reset(mreft_right);
   rox_dynvec_point2d_float_reset(mrefp_left);
   rox_dynvec_point2d_float_reset(mrefp_right);
   rox_dynvec_point3d_float_usecells(mreft_left, nb_points);
   rox_dynvec_point3d_float_usecells(mreft_right, nb_points);
   rox_dynvec_point2d_float_usecells(mrefp_left, nb_points);
   rox_dynvec_point2d_float_usecells(mrefp_right, nb_points);

   rox_array2d_double_fillunit(eye44);

   error = rox_array2d_double_mulmatmat(pose_right, rTl, pose);
   ROX_ERROR_CHECK_TERMINATE ( error );


   rox_point3d_float_transform(mreft_left->data, pose, references->data, references->used);
   rox_point2d_float_project(mrefp_left->data, mreft_left->data, calib_left, mreft_left->used);
   rox_point3d_float_transform(mreft_right->data, pose_right, references->data, references->used);
   rox_point2d_float_project(mrefp_right->data, mreft_right->data, calib_right, mreft_right->used);

   for (idpt = 0; idpt < references->used; idpt++)
   {
      Rox_Double dist;
      Rox_Double du, dv;
      Rox_Sint scale;
      Rox_Double w;

      scale = (Rox_Sint)( pow(2.0, levels->data[idpt]) );

      maxdist = maxdist_prefilter * scale;
      w = weights->data[idpt] / scale;

      du = mrefp_left->data[idpt].u - measures_left->data[idpt].u;
      dv = mrefp_left->data[idpt].v - measures_left->data[idpt].v;
      dist = sqrt(du*du + dv*dv);

      if (dist > maxdist) continue;

      du = mrefp_right->data[idpt].u - measures_right->data[idpt].u;
      dv = mrefp_right->data[idpt].v - measures_right->data[idpt].v;
      dist = sqrt(du*du + dv*dv);

      if (dist > maxdist) continue;

      rox_dynvec_point3d_float_append(references_filtered, &references->data[idpt]);
      rox_dynvec_point3d_float_append(mreft_left_filtered, &mreft_left->data[idpt]);
      rox_dynvec_point3d_float_append(mreft_right_filtered, &mreft_right->data[idpt]);
      rox_dynvec_point2d_float_append(mrefp_left_filtered, &mrefp_left->data[idpt]);
      rox_dynvec_point2d_float_append(mrefp_right_filtered, &mrefp_right->data[idpt]);
      rox_dynvec_point2d_float_append(measures_left_filtered, &measures_left->data[idpt]);
      rox_dynvec_point2d_float_append(measures_right_filtered, &measures_right->data[idpt]);
      rox_dynvec_double_append(weights_filtered, &w);
   }

   nb_points = mrefp_right_filtered->used;
   if (nb_points < 4) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error)}

   error = rox_array2d_double_new(&dist1, nb_points, 1); ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&dist2, nb_points, 1); ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&diff, nb_points * 2, 1); ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&weight, nb_points, 1); ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&work1, nb_points, 1); ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&work2, nb_points, 1); ROX_ERROR_CHECK_TERMINATE ( error );

   sqsum1 = 0.0;
   sqsum2 = 0.0;
   invcount = 1.0 / (Rox_Double)(nb_points * 2);
   
   error = rox_array2d_double_get_data_pointer ( &dw, weight);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (iter = 0; iter < max_iter; iter++)
   {
      // Compute right pose
      error = rox_array2d_double_mulmatmat(pose_right, rTl, pose);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_point3d_float_transform(mreft_left_filtered->data, pose, references_filtered->data, references_filtered->used); 
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_point2d_float_project(mrefp_left_filtered->data, mreft_left_filtered->data, calib_left, mreft_left_filtered->used); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_point3d_float_transform(mreft_right_filtered->data, pose_right, references_filtered->data, references_filtered->used); 
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_point2d_float_project(mrefp_right_filtered->data, mreft_right_filtered->data, calib_right, mreft_right_filtered->used); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_point2d_double_distance_point2d_float(dist1, mrefp_left_filtered, measures_left_filtered); 
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_array2d_point2d_double_distance_point2d_float(dist2, mrefp_right_filtered, measures_right_filtered); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_tukey_bounded(weight, work1, work2, dist1, 2.0, DBL_MAX); ROX_ERROR_CHECK_TERMINATE(error)
      for (idpt = 0; idpt < nb_points; idpt++)
      {
        dw[idpt] *= weights_filtered->data[idpt];
      }

      // Compute both jacobians
      error = rox_array2d_point2d_double_difference_point2d_float(diff, mrefp_left_filtered, measures_left_filtered); 
      ROX_ERROR_CHECK_TERMINATE ( error );
      error = rox_jacobian_se3_from_stereo_points_pixels_weighted_premul_float(JtJ_left, Jtf_left, diff, weight, references_filtered, calib_left, pose, eye44); 
      ROX_ERROR_CHECK_TERMINATE ( error );
      error = rox_vvs_score(&sqsum2, weight, dist1); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_tukey_bounded(weight, work1, work2, dist2, 2.0, DBL_MAX); 
      ROX_ERROR_CHECK_TERMINATE(error)
      for (idpt = 0; idpt < nb_points; idpt++)
      {
        dw[idpt] *= weights_filtered->data[idpt];
      }

      error = rox_array2d_point2d_double_difference_point2d_float(diff, mrefp_right_filtered, measures_right_filtered); ROX_ERROR_CHECK_TERMINATE(error)
      error = rox_jacobian_se3_from_stereo_points_pixels_weighted_premul_float(JtJ_right, Jtf_right, diff, weight, references_filtered, calib_right, pose, rTl); ROX_ERROR_CHECK_TERMINATE(error)
      error = rox_vvs_score(&lsqsum, weight, dist2); ROX_ERROR_CHECK_TERMINATE(error)
      sqsum2 += lsqsum;

      // Exit criteria
      delta = fabs(sqsum2 - sqsum1) * invcount;
      sqsum1 = sqsum2;
      if (delta < 1e-4 && iter > 5) break;

      error = rox_array2d_double_add(JtJ, JtJ_left, JtJ_right); ROX_ERROR_CHECK_TERMINATE(error)
      error = rox_array2d_double_add(Jtf, Jtf_left, Jtf_right); ROX_ERROR_CHECK_TERMINATE(error)
      error = rox_jacobian_se3_stereo_solver(pose, JtJ, Jtf);  ROX_ERROR_CHECK_TERMINATE(error)
   }

   error = ROX_ERROR_NONE;
   // Destroy buffers

function_terminate:

   rox_dynvec_point2d_float_del(&mrefp_left);
   rox_dynvec_point3d_float_del(&mreft_left);
   rox_dynvec_point2d_float_del(&mrefp_right);
   rox_dynvec_point3d_float_del(&mreft_right);
   rox_dynvec_point3d_float_del(&references_filtered);
   rox_dynvec_point2d_float_del(&measures_left_filtered);
   rox_dynvec_point2d_float_del(&measures_right_filtered);
   rox_dynvec_point2d_float_del(&mrefp_left_filtered);
   rox_dynvec_point3d_float_del(&mreft_left_filtered);
   rox_dynvec_point2d_float_del(&mrefp_right_filtered);
   rox_dynvec_point3d_float_del(&mreft_right_filtered);
   rox_dynvec_double_del(&weights_filtered);
   rox_array2d_double_del(&dist1);
   rox_array2d_double_del(&dist2);
   rox_array2d_double_del(&diff);
   rox_array2d_double_del(&weight);
   rox_array2d_double_del(&work1);
   rox_array2d_double_del(&work2);
   rox_array2d_double_del(&JtJ);
   rox_array2d_double_del(&JtJ_left);
   rox_array2d_double_del(&JtJ_right);
   rox_array2d_double_del(&Jtf);
   rox_array2d_double_del(&Jtf_left);
   rox_array2d_double_del(&Jtf_right);
   rox_array2d_double_del(&pose_right);
   rox_array2d_double_del(&eye44);

   return error;
}
