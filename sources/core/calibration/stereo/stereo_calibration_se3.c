//==============================================================================
//
//    OPENROX   : File stereo_calibration_se3.c
//
//    Contents  : Implementation of stereo_calibration_se3 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "stereo_calibration_se3.h"
#include "stereo_calibration_se3_struct.h"

#include <baseproc/array/multiply/mulmatmat.h>
#include <baseproc/array/scale/scale.h>
#include <baseproc/array/fill/fillval.h>
#include <baseproc/array/fill/fillunit.h>
#include <baseproc/array/inverse/svdinverse.h>
#include <baseproc/array/solve/svd_solve.h>
#include <baseproc/geometry/point/point2d_projection_from_point3d.h>
#include <baseproc/array/minmax/minmax.h>
#include <baseproc/array/median/median.h>
#include <baseproc/geometry/transforms/transform_tools.h>
#include <baseproc/geometry/point/point2d_projection_from_point3d_transform.h>
#include <baseproc/geometry/point/point2d_matsl3_transform.h>
#include <baseproc/maths/maths_macros.h>
//#include <baseproc/maths/linalg/generators/algse3.h>
#include <baseproc/maths/linalg/matse3.h>
#include <baseproc/maths/linalg/generators/algut3.h>
#include <baseproc/calculus/jacobians/jacobian_perspective_stereo_calibration.h>
#include <inout/system/print.h>
#include <inout/system/errors_print.h>
#include <baseproc/maths/maths_macros.h>
#include <stdio.h>

#define threshold_reprojection 1.5

Rox_ErrorCode rox_calibration_stereo_perspective_se3_new(Rox_Calibration_Stereo_Perspective_SE3 * obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Calibration_Stereo_Perspective_SE3 ret = NULL;


   if(!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *obj = NULL;

   ret = (Rox_Calibration_Stereo_Perspective_SE3)rox_memory_allocate(sizeof(*ret), 1);
   if(!ret) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   // Set pointers to NULL
   ret->model = NULL;
   ret->lpoints = 0;
   ret->rpoints = 0;
   ret->lposes = 0;
   ret->rposes = 0;
   ret->lhomographies = 0;
   ret->rhomographies = 0;
   ret->Kl = 0;
   ret->Kr = 0;
   ret->rTl = 0;
   ret->rTlposes = 0;
   ret->valid_flags = 0;

   error = rox_dynvec_point3d_double_new(&ret->model, 50); ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_objset_dynvec_point2d_double_new(&ret->lpoints, 20); ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_objset_dynvec_point2d_double_new(&ret->rpoints, 20); ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_objset_array2d_double_new(&ret->lposes, 10); ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_objset_array2d_double_new(&ret->rposes, 10); ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_objset_array2d_double_new(&ret->rTlposes, 10); ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_objset_array2d_double_new(&ret->lhomographies, 10); ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_objset_array2d_double_new(&ret->rhomographies, 10); ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_objset_array2d_double_new(&ret->rTlposes, 10); ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_dynvec_uint_new(&ret->valid_flags, 20); ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_new(&ret->Kl, 3, 3);ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_new(&ret->Kr, 3, 3);ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_new(&ret->rTl, 4, 4);ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_fillunit(ret->Kl);ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_fillunit(ret->Kr);ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_fillunit(ret->rTl);ROX_ERROR_CHECK_TERMINATE(error)

   *obj = ret;

function_terminate:
   if(error) rox_calibration_stereo_perspective_se3_del(&ret);

   return error;
}

Rox_ErrorCode rox_calibration_stereo_perspective_se3_del(Rox_Calibration_Stereo_Perspective_SE3 * obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Calibration_Stereo_Perspective_SE3 todel = NULL;


   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   todel = *obj;
   *obj = NULL;

   if (!todel) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_dynvec_point3d_double_del(&todel->model);
   rox_objset_dynvec_point2d_double_del(&todel->lpoints);
   rox_objset_dynvec_point2d_double_del(&todel->rpoints);
   rox_objset_array2d_double_del(&todel->lposes);
   rox_objset_array2d_double_del(&todel->rposes);
   rox_objset_array2d_double_del(&todel->lhomographies);
   rox_objset_array2d_double_del(&todel->rhomographies);
   rox_objset_array2d_double_del(&todel->rTlposes);
   rox_dynvec_uint_del(&todel->valid_flags);

   rox_array2d_double_del(&todel->Kl);
   rox_array2d_double_del(&todel->Kr);
   rox_array2d_double_del(&todel->rTl);

function_terminate:
   return error;
}

Rox_ErrorCode rox_calibration_stereo_perspective_se3_set_model_points (
   Rox_Calibration_Stereo_Perspective_SE3 obj, Rox_Point3D_Double model, Rox_Uint count)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if(!obj || !model) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   // Reset model
   rox_dynvec_point3d_double_reset(obj->model);

   for (Rox_Uint i = 0; i < count; i++)
   {

      error = rox_dynvec_point3d_double_append(obj->model, &model[i]); 
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_calibration_stereo_perspective_se3_add_current_points(Rox_Calibration_Stereo_Perspective_SE3 obj, Rox_Point2D_Double  left_pts, Rox_Point2D_Double  right_pts, Rox_Uint count)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_DynVec_Point2D_Double lpts = NULL;
   Rox_DynVec_Point2D_Double rpts = NULL;

   if(!obj || !left_pts || !right_pts) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   error = rox_dynvec_point2d_double_new(&lpts, 50); ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_dynvec_point2d_double_new(&rpts, 50); ROX_ERROR_CHECK_TERMINATE(error)

   for (Rox_Uint i = 0; i < count; i++)
   {
      Rox_Point2D_Double_Struct lcur = left_pts[i];
      Rox_Point2D_Double_Struct rcur = right_pts[i];

      error = rox_dynvec_point2d_double_append(lpts, &lcur); ROX_ERROR_CHECK_TERMINATE(error)
      error = rox_dynvec_point2d_double_append(rpts, &rcur); ROX_ERROR_CHECK_TERMINATE(error)
   }

   error = rox_objset_dynvec_point2d_double_append(obj->lpoints, lpts);ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_objset_dynvec_point2d_double_append(obj->rpoints, rpts);ROX_ERROR_CHECK_TERMINATE(error)

function_terminate:
   if(error)
   {
      rox_dynvec_point2d_double_del(&lpts);
      rox_dynvec_point2d_double_del(&rpts);
   }

   return error;
}

Rox_ErrorCode rox_calibration_stereo_perspective_se3_add_current_homographies(Rox_Calibration_Stereo_Perspective_SE3 obj, Rox_Array2D_Double Gl, Rox_Array2D_Double Gr)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double lpose = 0, rpose = 0, rTl = 0;
   Rox_Array2D_Double lhomography = 0, rhomography = 0;
   Rox_Uint valid_flag = 1;

   if(!obj || !Gl || !Gr) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   error = rox_array2d_double_new(&lpose, 4, 4);ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_new(&rpose, 4, 4);ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_new(&rTl, 4, 4);ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_new(&lhomography, 3, 3);ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_new(&rhomography, 3, 3);ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_copy(lhomography, Gl);ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_copy(rhomography, Gr);ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_objset_array2d_double_append(obj->lhomographies, lhomography); ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_objset_array2d_double_append(obj->rhomographies, rhomography); ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_objset_array2d_double_append(obj->lposes, lpose); ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_objset_array2d_double_append(obj->rposes, rpose); ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_objset_array2d_double_append(obj->rTlposes, rTl); ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_dynvec_uint_append(obj->valid_flags, &valid_flag);ROX_ERROR_CHECK_TERMINATE(error)

function_terminate:
   if(error)
   {
      rox_array2d_double_del(&lpose);
      rox_array2d_double_del(&rpose);

      rox_array2d_double_del(&lhomography);
      rox_array2d_double_del(&rhomography);
   }

   return error;
}

Rox_ErrorCode rox_calibration_stereo_perspective_se3_set_intrinsics(Rox_Calibration_Stereo_Perspective_SE3 obj, Rox_Array2D_Double Kl, Rox_Array2D_Double Kr)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if(!obj || !Kl || !Kr) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   error = rox_array2d_double_copy(obj->Kl, Kl); ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_copy(obj->Kr, Kr); ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_calibration_stereo_perspective_se3_process_nolinear(Rox_Calibration_Stereo_Perspective_SE3 obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint nbimages, nbpoints;
   Rox_Uint Tdll = 6, iter = 10;
   Rox_Uint Jcols, Jrows, start_col, start_row;
   Rox_Double lambda = 0.9;
   Rox_Array2D_Double J = 0, JTrl = 0, JTl = 0, JTr = 0, rTo = 0;
   Rox_Array2D_Double b = 0, x = 0, xTrl = 0, xT =0;
   Rox_Array2D_Double lTo;

   Rox_Point2D_Double cur_left = NULL;
   Rox_Point2D_Double cur_right = NULL;
   Rox_Double *zl = 0, *zr = 0;


   if(!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   nbimages = obj->lposes->used;
   nbpoints = obj->model->used;

   // Allocate J
   Jrows = 4*nbpoints * nbimages;
   Jcols = Tdll + Tdll * nbimages; // Trl - T(1) - ... - T(nbimages)

   error = rox_array2d_double_new(&rTo, 4, 4);
   ROX_ERROR_CHECK_TERMINATE(error)

   
   error = rox_array2d_double_new(&J, Jrows, Jcols); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&b, 4*nbpoints*nbimages, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&x, Jcols, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   cur_left  = (Rox_Point2D_Double )rox_memory_allocate(sizeof(*cur_left), nbpoints);
   if(!cur_left)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   cur_right = (Rox_Point2D_Double )rox_memory_allocate(sizeof(*cur_right), nbpoints);
   if (!cur_right)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   zl = (Rox_Double*)rox_memory_allocate(sizeof(*zl), nbpoints);
   if (!zl)
   {  error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   zr = (Rox_Double*)rox_memory_allocate(sizeof(*zr), nbpoints);
   if (!zl)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   for (Rox_Uint k = 0; k < iter; k++)
   {
      // Reset J
      error = rox_array2d_double_fillval(J, 0.0);
      ROX_ERROR_CHECK_TERMINATE(error)
      error = rox_array2d_double_fillval(x, 0.0);
      ROX_ERROR_CHECK_TERMINATE(error)
      error = rox_array2d_double_fillval(b, 0.0);
      ROX_ERROR_CHECK_TERMINATE(error)

      for(Rox_Uint i = 0; i < nbimages; i++)
      {
         if(obj->valid_flags->data[i] == 0) continue;

         lTo = obj->lposes->data[i];

         // Get Jacobian subviews
         start_col = 0;
         start_row = 4 * nbpoints * i + 2 *nbpoints;
         error = rox_array2d_double_new_subarray2d(&JTrl,J, start_row, start_col, 2*nbpoints, Tdll);
         ROX_ERROR_CHECK_TERMINATE(error)

         start_col = Tdll + i * Tdll;
         start_row = 4 * nbpoints * i;
         error = rox_array2d_double_new_subarray2d(&JTl, J, start_row, start_col, 2*nbpoints, Tdll);
         ROX_ERROR_CHECK_TERMINATE(error)

         start_col = Tdll + i * Tdll;
         start_row = 4 * nbpoints * i + 2 *nbpoints;
         error = rox_array2d_double_new_subarray2d(&JTr, J, start_row, start_col, 2*nbpoints, Tdll);
         ROX_ERROR_CHECK_TERMINATE(error)

         // rTo = rTl * lTo
         error = rox_array2d_double_mulmatmat(rTo, obj->rTl, lTo);
         ROX_ERROR_CHECK_TERMINATE(error)

         error = rox_point2d_double_transform_project(cur_left, zl, obj->Kl, lTo, obj->model->data, nbpoints);
         ROX_ERROR_CHECK_TERMINATE(error)
         error = rox_point2d_double_transform_project(cur_right,zr, obj->Kr, rTo, obj->model->data, nbpoints);
         ROX_ERROR_CHECK_TERMINATE(error)

         // compute jacobians
         error = rox_jacobian_perspective_stereo_calibration_pose_intercamera(JTrl, obj->Kr, obj->rTl, lTo, rTo, obj->model->data, nbpoints);
         ROX_ERROR_CHECK_TERMINATE(error)
         error = rox_jacobian_perspective_stereo_calibration_pose(JTl, obj->Kl, lTo, obj->model->data, cur_left, zl, nbpoints);
         ROX_ERROR_CHECK_TERMINATE(error)
         error = rox_jacobian_perspective_stereo_calibration_pose(JTr, obj->Kr, rTo, obj->model->data, cur_right, zr, nbpoints);
         ROX_ERROR_CHECK_TERMINATE(error)

         // Build vector b
         for(Rox_Uint j = 0; j < nbpoints; j++)
         {
               Rox_Uint row = 4 * nbpoints * i + j * 2;

               error = rox_array2d_double_set_value(b, row  , 0, cur_left[j].u  - obj->lpoints->data[i]->data[j].u);
               ROX_ERROR_CHECK_TERMINATE(error)
               error = rox_array2d_double_set_value(b, row+1, 0, cur_left[j].v  - obj->lpoints->data[i]->data[j].v);
               ROX_ERROR_CHECK_TERMINATE(error)
         }
         for(Rox_Uint j = 0; j < nbpoints; j++)
         {
            Rox_Uint row = 4 * nbpoints * i + 2 * nbpoints + j * 2;

            error = rox_array2d_double_set_value(b, row  , 0, cur_right[j].u - obj->rpoints->data[i]->data[j].u);
            ROX_ERROR_CHECK_TERMINATE(error)
            error = rox_array2d_double_set_value(b, row+1, 0, cur_right[j].v - obj->rpoints->data[i]->data[j].v);
            ROX_ERROR_CHECK_TERMINATE(error)
         }

         // Free subviews
         rox_array2d_double_del(&JTr);
         rox_array2d_double_del(&JTl);
         rox_array2d_double_del(&JTrl);
      }

      // Solve x = pinv(J)*b
      error = rox_svd_solve(x, J, b);
      ROX_ERROR_CHECK_TERMINATE(error)
      error = rox_array2d_double_scale(x, x, -lambda);
      ROX_ERROR_CHECK_TERMINATE(error)
      error = rox_array2d_double_new_subarray2d(&xTrl , x, 0, 0, Tdll, 1);
      ROX_ERROR_CHECK_TERMINATE(error)

      // Update poses
      error = rox_matse3_update_right(obj->rTl, xTrl); ROX_ERROR_CHECK_TERMINATE(error)

      for(Rox_Uint i = 0; i < nbimages; i++)
      {
         if(obj->valid_flags->data[i] == 0) continue;
         error = rox_array2d_double_new_subarray2d(&xT, x, Tdll + Tdll * i, 0, Tdll, 1);
         ROX_ERROR_CHECK_TERMINATE(error)
         error = rox_matse3_update_right(obj->lposes->data[i], xT);
         ROX_ERROR_CHECK_TERMINATE(error)
         rox_array2d_double_del(&xT);
      }
   }

function_terminate:
   rox_array2d_double_del(&JTr);
   rox_array2d_double_del(&JTl);
   rox_array2d_double_del(&JTrl);
   rox_array2d_double_del(&J);
   rox_array2d_double_del(&xT);
   rox_array2d_double_del(&b);
   rox_array2d_double_del(&x);
   rox_array2d_double_del(&xTrl);

   rox_memory_delete(cur_left);
   rox_memory_delete(cur_right);
   rox_memory_delete(zl);
   rox_memory_delete(zr);

   return error;
}

Rox_ErrorCode rox_calibration_stereo_perspective_se3_make(Rox_Calibration_Stereo_Perspective_SE3 obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double min, max, mean, med, std;
   int recalibrate = 0;

   Rox_Array2D_Double rTl = 0;

   if(!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   if(obj->lposes->used == 0 || obj->rposes->used == 0) {error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE(error)}

   error = rox_array2d_double_new(&rTl, 4, 4); ROX_ERROR_CHECK_TERMINATE(error)

   // Check homographies
   error = rox_calibration_stereo_perspective_se3_check_homographies(obj); ROX_ERROR_CHECK_TERMINATE(error)

   // Build all poses
   for (Rox_Uint i = 0; i < obj->lposes->used; i++)
   {
      // Check flags
      if(obj->valid_flags->data[i] == 0) continue;

      error = rox_transformtools_build_pose_intermodel(obj->lposes->data[i], obj->lhomographies->data[i], obj->Kl);
      ROX_ERROR_CHECK_TERMINATE(error)

      error = rox_transformtools_build_pose_intermodel(obj->rposes->data[i], obj->rhomographies->data[i], obj->Kr);
      ROX_ERROR_CHECK_TERMINATE(error)
   }

   // Compute intercamera pose
   error = rox_calibration_stereo_perspective_se3_compute_intercamera_pose(obj);
   ROX_ERROR_CHECK_TERMINATE(error)

   // Make a copy
   error = rox_array2d_double_copy(rTl, obj->rTl);
   ROX_ERROR_CHECK_TERMINATE(error)

   // refine estimation
   error = rox_calibration_stereo_perspective_se3_process_nolinear(obj); ROX_ERROR_CHECK_TERMINATE(error)

   rox_log("Stats\n");

   // Get stats
   for (Rox_Uint i = 0; i < obj->lhomographies->used; i++)
   {
      if(obj->valid_flags->data[i] == 0) continue;

      error = rox_calibration_stereo_perspective_se3_get_left_statistics(&min, &max, &mean, &med, &std, obj, i);
      ROX_ERROR_CHECK_TERMINATE(error)

      rox_log("Left: %f, %f, %f, %f, %f\n", min , max, mean, med, std);

      if(max > threshold_reprojection)
      {
         obj->valid_flags->data[i] = 0;
         recalibrate = 1;
         rox_log("Removed image %d\n", i);
         rox_log("Statistics are : %f, %f, %f, %f, %f\n", min , max, mean, med, std);
      }

      error = rox_calibration_stereo_perspective_se3_get_right_statistics(&min, &max, &mean, &med, &std, obj, i);
      ROX_ERROR_CHECK_TERMINATE(error)

      rox_log("Right: %f, %f, %f, %f, %f\n", min , max, mean, med, std);
      if(max > threshold_reprojection)
      {
         obj->valid_flags->data[i] = 0;
         recalibrate = 1;
         rox_log("Removed image %d\n", i);
         rox_log("Statistics are : %f, %f, %f, %f, %f\n", min, max, mean, med, std);
      }
   }

   if(recalibrate)
   {
      rox_log("recalibration...\n");

      // Restore data
      error = rox_array2d_double_copy(obj->rTl, rTl); ROX_ERROR_CHECK_TERMINATE(error)

      error = rox_calibration_stereo_perspective_se3_process_nolinear(obj); ROX_ERROR_CHECK_TERMINATE(error)

      // Get stats
      for (Rox_Uint i = 0; i < obj->lhomographies->used; i++)
      {
         if(obj->valid_flags->data[i] == 0) continue;

         error = rox_calibration_stereo_perspective_se3_get_left_statistics(&min, &max, &mean, &med, &std, obj, i);
         ROX_ERROR_CHECK_TERMINATE(error)

         rox_log("Left: %f, %f, %f, %f, %f\n", min , max, mean, med, std);

         if(max > threshold_reprojection)
         {
            error = ROX_ERROR_PROCESS_FAILED;
            ROX_ERROR_CHECK_TERMINATE(error)
         }

         error = rox_calibration_stereo_perspective_se3_get_right_statistics(&min, &max, &mean, &med, &std, obj, i);
         ROX_ERROR_CHECK_TERMINATE(error)

         rox_log("Right: %f, %f, %f, %f, %f\n", min , max, mean, med, std);
         if(max > threshold_reprojection)
         {
            error = ROX_ERROR_PROCESS_FAILED;
            ROX_ERROR_CHECK_TERMINATE(error)
         }
      }
   }

function_terminate:
   rox_array2d_double_del(&rTl);

   return error;
}

Rox_ErrorCode rox_calibration_stereo_perspective_se3_get_results(Rox_Array2D_Double pose, Rox_Calibration_Stereo_Perspective_SE3 obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if(!pose || !obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   error = rox_array2d_double_copy(pose, obj->rTl);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_calibration_stereo_perspective_se3_print_statistics(Rox_Calibration_Stereo_Perspective_SE3 obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double norm = 0, pose = 0, lTr = 0;
   Rox_Uint nbimages, nbpoints;

   Rox_Point2D_Double  reproj = 0;
   Rox_Double **dn;

   if(!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   nbimages = obj->lposes->used;
   nbpoints = obj->model->used;

   reproj = (Rox_Point2D_Double )rox_memory_allocate(sizeof(*reproj), nbpoints);
   if (!reproj)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_new(&norm, nbpoints, 1); ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&pose, 4, 4); ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&lTr, 4, 4); ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_svdinverse(lTr, obj->rTl); ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer_to_pointer(&dn, norm);ROX_ERROR_CHECK_TERMINATE ( error );

   // Right
   rox_log("Right \n");
   for(Rox_Uint i = 0; i < nbimages; i++)
   {
      Rox_Double min, max, mean, sum, var, med, u1, v1, u2, v2;

      if(obj->valid_flags->data[i] == 0) continue;

      // rTm = rTl * lTm
      error = rox_array2d_double_mulmatmat(pose, obj->rTl, obj->lposes->data[i]); ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_point3d_double_transform_project(reproj, pose, obj->Kr, obj->model->data, nbpoints);
      ROX_ERROR_CHECK_TERMINATE(error)

      // Compute norm
      for(Rox_Uint j = 0; j < nbpoints; j++)
      {
         // Compute norm
         u1 = reproj[j].u; v1 = reproj[j].v;
         u2 = obj->rpoints->data[i]->data[j].u; v2 = obj->rpoints->data[i]->data[j].v;

         dn[j][0] = sqrt((u1 - u2)*(u1 - u2) + (v1 - v2)*(v1 - v2));
      }

      // make stats
      error = rox_array2d_double_minmax(&min, &max, norm); ROX_ERROR_CHECK_TERMINATE(error)
      error = rox_array2d_double_median(&med, norm);ROX_ERROR_CHECK_TERMINATE(error)

      sum = 0;
      for(Rox_Uint j = 0; j < nbpoints; j++)
      {
         sum += dn[j][0];
      }

      mean = sum / (Rox_Double)nbpoints;

      sum = 0;
      for(Rox_Uint j = 0; j < nbpoints; j++)
      {
         sum += (dn[j][0] - mean) * (dn[j][0] - mean);
      }
      var = sum / (Rox_Double)nbpoints;

      rox_log("min %f, max %f, mean %f, variance %f, median %f\n", min, max, mean, var, med);
   }

   // Left
   rox_log("Left \n");
   for(Rox_Uint i = 0; i < nbimages; i++)
   {
      Rox_Double min, max, mean, sum, var, med, u1, v1, u2, v2;

      if(obj->valid_flags->data[i] == 0) continue;

      error = rox_point3d_double_transform_project(reproj, obj->lposes->data[i], obj->Kl, obj->model->data, nbpoints);
      ROX_ERROR_CHECK_TERMINATE(error)

      // Compute norm
      for(Rox_Uint j = 0; j < nbpoints; j++)
      {
         // Compute norm
         u1 = reproj[j].u; v1 = reproj[j].v;
         u2 = obj->lpoints->data[i]->data[j].u; v2 = obj->lpoints->data[i]->data[j].v;

         dn[j][0] = sqrt((u1 - u2)*(u1 - u2) + (v1 - v2)*(v1 - v2));
      }

      // make stats
      error = rox_array2d_double_minmax(&min, &max, norm); ROX_ERROR_CHECK_TERMINATE(error)
      error = rox_array2d_double_median(&med, norm);ROX_ERROR_CHECK_TERMINATE(error)

      sum = 0;
      for(Rox_Uint j = 0; j < nbpoints; j++)
      {
         sum += dn[j][0];
      }

      mean = sum / (Rox_Double)nbpoints;

      sum = 0;
      for(Rox_Uint j = 0; j < nbpoints; j++)
      {
         sum += (dn[j][0] - mean) * (dn[j][0] - mean);
      }
      var = sum / (Rox_Double)nbpoints;

      rox_log("min %f, max %f, mean %f, variance %f, median %f\n", min, max, mean, var, med);
   }

   function_terminate:
   rox_memory_delete(reproj);
   rox_array2d_double_del(&norm);
   rox_array2d_double_del(&pose);
   rox_array2d_double_del(&lTr);

   return error;
}

Rox_ErrorCode rox_calibration_stereo_perspective_se3_check_homographies(Rox_Calibration_Stereo_Perspective_SE3 obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double lnorm = 0, rnorm = 0;
   Rox_Uint nbimages, nbpoints;

   Rox_Point2D_Double lreproj = NULL, rreproj = NULL;
   Rox_Point2D_Double model = NULL;
   Rox_Double **ldn, **rdn;


   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   nbimages = obj->lhomographies->used;
   nbpoints = obj->model->used;

   lreproj = (Rox_Point2D_Double )rox_memory_allocate(sizeof(*lreproj), nbpoints);
   if(!lreproj)
   {
      error = ROX_ERROR_NULL_POINTER;
      ROX_ERROR_CHECK_TERMINATE(error)
   }

   rreproj = (Rox_Point2D_Double ) rox_memory_allocate(sizeof(*rreproj), nbpoints);
   if(!rreproj)
   {
      error = ROX_ERROR_NULL_POINTER;
      ROX_ERROR_CHECK_TERMINATE(error)
   }

   model = (Rox_Point2D_Double )rox_memory_allocate(sizeof(*model), nbpoints);
   if (!model)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_new(&lnorm, nbpoints, 1); ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_get_data_pointer_to_pointer(&ldn, lnorm); ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&rnorm, nbpoints, 1); ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_get_data_pointer_to_pointer(&rdn, rnorm); ROX_ERROR_CHECK_TERMINATE ( error );

   // Build model
   for(Rox_Uint i = 0; i < nbpoints; i++)
   {
      model[i].u = obj->model->data[i].X;
      model[i].v = obj->model->data[i].Y;
   }

   for(Rox_Uint i = 0; i < nbimages; i++)
   {
      Rox_Double lmin, lmax, lmean, lsum, lvar, lmed, lu1, lv1, lu2, lv2;
      Rox_Double rmin, rmax, rmean, rsum, rvar, rmed, ru1, rv1, ru2, rv2;

      error = rox_point2d_double_homography(lreproj, model, obj->lhomographies->data[i], nbpoints);
      ROX_ERROR_CHECK_TERMINATE(error)

      error = rox_point2d_double_homography(rreproj, model, obj->rhomographies->data[i], nbpoints);
      ROX_ERROR_CHECK_TERMINATE(error)

      // Compute norm
      for(Rox_Uint j = 0; j < nbpoints; j++)
      {
         // Compute norm
         // Left
         lu1 = lreproj[j].u; lv1 = lreproj[j].v;
         lu2 = obj->lpoints->data[i]->data[j].u; lv2 = obj->lpoints->data[i]->data[j].v;

         ldn[j][0] = sqrt((lu1 - lu2)*(lu1 - lu2) + (lv1 - lv2)*(lv1 - lv2));

         // Right
         ru1 = rreproj[j].u; rv1 = rreproj[j].v;
         ru2 = obj->rpoints->data[i]->data[j].u; rv2 = obj->rpoints->data[i]->data[j].v;

         rdn[j][0] = sqrt((ru1 - ru2)*(ru1 - ru2) + (rv1 - rv2)*(rv1 - rv2));
      }

      // make stats
      error = rox_array2d_double_minmax(&lmin, &lmax, lnorm); ROX_ERROR_CHECK_TERMINATE(error)
      error = rox_array2d_double_median(&lmed, lnorm);ROX_ERROR_CHECK_TERMINATE(error)

      error = rox_array2d_double_minmax(&rmin, &rmax, rnorm); ROX_ERROR_CHECK_TERMINATE(error)
      error = rox_array2d_double_median(&rmed, rnorm);ROX_ERROR_CHECK_TERMINATE(error)

      lsum = 0;
      rsum = 0;
      for(Rox_Uint j = 0; j < nbpoints; j++)
      {
         lsum += ldn[j][0];
         rsum += rdn[j][0];
      }

      lmean = lsum / (Rox_Double)nbpoints;
      rmean = rsum / (Rox_Double)nbpoints;

      lsum = 0;
      rsum = 0;
      for(Rox_Uint j = 0; j < nbpoints; j++)
      {
         lsum += (ldn[j][0] - lmean) * (ldn[j][0] - lmean);
         rsum += (rdn[j][0] - rmean) * (rdn[j][0] - rmean);
      }
      lvar = sqrt(lsum / (Rox_Double)nbpoints);
      rvar = sqrt(rsum / (Rox_Double)nbpoints);

      if(lmax > threshold_reprojection)
      {
         rox_log("Stereo calibration(homographie): removed left image %d\n", i);
         rox_log("Statistics are: %f, %f, %f, %f, %f\n", lmin, lmax, lmean, lmed, lvar);
         obj->valid_flags->data[i] = 0;
      }
      rox_log("Left Statistics are: %f, %f, %f, %f, %f\n", lmin, lmax, lmean, lmed, lvar);
      if(rmax > threshold_reprojection)
      {
         rox_log("Stereo calibration(homographie): removed right image %d\n", i);
         rox_log("Statistics are: %f, %f, %f, %f, %f\n", rmin, rmax, rmean, rmed, rvar);
         obj->valid_flags->data[i] = 0;
      }
      rox_log("Right Statistics are: %f, %f, %f, %f, %f\n", rmin, rmax, rmean, rmed, rvar);
   }

   function_terminate:
   rox_memory_delete(lreproj);
   rox_memory_delete(rreproj);
   rox_memory_delete(model);
   rox_array2d_double_del(&lnorm);
   rox_array2d_double_del(&rnorm);

   return error;
}

Rox_ErrorCode rox_calibration_stereo_perspective_se3_compute_intercamera_pose(Rox_Calibration_Stereo_Perspective_SE3 obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double mTl = 0, R = 0, t = 0;
   Rox_Array2D_Double tx = 0, ty = 0, tz = 0, rx = 0, ry = 0, rz = 0, nt = 0, nr = 0;
   Rox_Double **dtx, **dty, **dtz, **drx, **dry, **drz, **dt, **dnt, **dnr;
   Rox_Double ax, ay, az, ang;
   Rox_Double med_ax, med_ay, med_az, med_tx, med_ty, med_tz;
   Rox_Double mean_tx, mean_ty, mean_tz, sum_tx = 0, sum_ty = 0, sum_tz = 0;
   Rox_Double mean_rx, mean_ry, mean_rz, sum_rx = 0, sum_ry = 0, sum_rz = 0;
   Rox_Double mean_nt, mean_nr, sum_nt = 0, sum_nr = 0, var_nt, var_nr;

   Rox_Uint nbpose;

   if(!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   nbpose = 0;

   for(Rox_Uint i = 0; i < obj->lposes->used; i++)
   {
      // valid images
      if(obj->valid_flags->data[i] == 1)
         nbpose++;
   }

   error = rox_array2d_double_new(&mTl, 4, 4); ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&tx, nbpose, 1); ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&ty, nbpose, 1); ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&tz, nbpose, 1); ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&rx, nbpose, 1); ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&ry, nbpose, 1); ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&rz, nbpose, 1); ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&nt, nbpose, 1); ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&nr, nbpose, 1); ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer_to_pointer(&dtx, tx); ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_get_data_pointer_to_pointer(&dty, ty); ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_get_data_pointer_to_pointer(&dtz, tz); ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_get_data_pointer_to_pointer(&drx, rx); ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_get_data_pointer_to_pointer(&dry, ry); ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_get_data_pointer_to_pointer(&drz, rz); ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_get_data_pointer_to_pointer(&dnt, nt); ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_get_data_pointer_to_pointer(&dnr, nr); ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint j = 0;

   // Build all rTl poses
   for(Rox_Uint i = 0; i < obj->lposes->used; i++)
   {
      // valid images
      if(obj->valid_flags->data[i] == 0) continue;


      error = rox_array2d_double_svdinverse(mTl, obj->lposes->data[i]); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_mulmatmat(obj->rTlposes->data[i], obj->rposes->data[i], mTl); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_new_subarray2d(&R, obj->rTlposes->data[i], 0, 0, 3, 3);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_new_subarray2d(&t, obj->rTlposes->data[i], 0, 3, 3, 1);
      ROX_ERROR_CHECK_TERMINATE ( error );


      error = rox_array2d_double_get_data_pointer_to_pointer(&dt, t); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_transformtools_axisangle_from_rotationmatrix(&ax, &ay, &az, &ang, R);
      ROX_ERROR_CHECK_TERMINATE ( error );

      drx[j][0] = ax * ang;
      dry[j][0] = ay * ang;
      drz[j][0] = az * ang;
      dtx[j][0] = dt[0][0];
      dty[j][0] = dt[1][0];
      dtz[j][0] = dt[2][0];

      sum_tx += dt[0][0];
      sum_ty += dt[1][0];
      sum_tz += dt[2][0];
      sum_rx += ax * ang;
      sum_ry += ay * ang;
      sum_rz += az * ang;

      j++;

      rox_array2d_double_del(&R);
      rox_array2d_double_del(&t);
   }

   // Build rTl
   error = rox_array2d_double_median(&med_ax, rx);ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_median(&med_ay, ry);ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_median(&med_az, rz);ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_median(&med_tx, tx);ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_median(&med_ty, ty);ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_median(&med_tz, tz);ROX_ERROR_CHECK_TERMINATE(error)

   ang = sqrt(med_ax*med_ax + med_ay*med_ay + med_az*med_az);
   ax = med_ax / ang;
   ay = med_ay / ang;
   az = med_az / ang;

   error = rox_matse3_set_axis_angle_translation(obj->rTl, ax, ay, az, ang, med_tx, med_ty, med_tz);
   ROX_ERROR_CHECK_TERMINATE ( error );

   mean_tx = sum_tx / (Rox_Double)nbpose;
   mean_ty = sum_ty / (Rox_Double)nbpose;
   mean_tz = sum_tz / (Rox_Double)nbpose;

   mean_rx = sum_rx / (Rox_Double)nbpose;
   mean_ry = sum_ry / (Rox_Double)nbpose;
   mean_rz = sum_rz / (Rox_Double)nbpose;

   // build norms
   j = 0;
   for(Rox_Uint i = 0; i < obj->rTlposes->used; i++)
   {
      Rox_Double diftx, difty, diftz, difrx, difry, difrz;

      // valid images
      if(obj->valid_flags->data[i] == 0) continue;

      // Translation
      error = rox_array2d_double_new_subarray2d(&R, obj->rTlposes->data[i], 0, 0, 3, 3);ROX_ERROR_CHECK_TERMINATE ( error );
      error = rox_array2d_double_new_subarray2d(&t, obj->rTlposes->data[i], 0, 3, 3, 1);ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_get_data_pointer_to_pointer(&dt, t); ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_transformtools_axisangle_from_rotationmatrix(&ax, &ay, &az, &ang, R);ROX_ERROR_CHECK_TERMINATE ( error );

      difrx = ax * ang - mean_rx;
      difry = ay * ang - mean_ry;
      difrz = az * ang - mean_rz;
      diftx = dt[0][0] - mean_tx;
      difty = dt[1][0] - mean_ty;
      diftz = dt[2][0] - mean_tz;

      dnt[j][0] = sqrt(diftx*diftx + difty*difty + diftz*diftz);
      dnr[j][0] = sqrt(difrx*difrx + difry*difry + difrz*difrz)*180/ROX_PI;

      sum_nt += dnt[j][0];
      sum_nr += dnr[j][0];

      j++;

      rox_array2d_double_del(&R);
      rox_array2d_double_del(&t);
   }

   mean_nr = sum_nr / (Rox_Double)nbpose;
   mean_nt = sum_nt / (Rox_Double)nbpose;

   sum_nr = 0;
   sum_nt = 0;
   for(Rox_Uint j = 0; j < nbpose; j++)
   {
      sum_nr += (dnr[j][0] - mean_nr) * (dnr[j][0] - mean_nr);
      sum_nt += (dnt[j][0] - mean_nt) * (dnt[j][0] - mean_nt);
   }
   var_nr = sqrt(sum_nr / (Rox_Double)(nbpose-1));
   var_nt = sqrt(sum_nt / (Rox_Double)(nbpose-1));

   // Check images
   j = 0;
   for(Rox_Uint i = 0; i < obj->rTlposes->used; i++)
   {
      // valid images
      if(obj->valid_flags->data[i] == 0) continue;

      if(dnt[j][0] > 3 * var_nt)
      {
         obj->valid_flags->data[i] = 0;
         rox_log("removed image %d\n", i);
      }

      if(dnr[j][0] > 3 * var_nr)
      {
         obj->valid_flags->data[i] = 0;
         rox_log("removed image %d\n", i);
      }
      j++;
   }

   function_terminate:
   rox_array2d_double_del(&R);
   rox_array2d_double_del(&t);
   rox_array2d_double_del(&mTl);
   rox_array2d_double_del(&tx);
   rox_array2d_double_del(&ty);
   rox_array2d_double_del(&tz);
   rox_array2d_double_del(&rx);
   rox_array2d_double_del(&ry);
   rox_array2d_double_del(&rz);
   rox_array2d_double_del(&nr);
   rox_array2d_double_del(&nt);

   return error;
}

Rox_ErrorCode rox_calibration_stereo_perspective_se3_get_left_statistics(Rox_Double *min, Rox_Double *max, Rox_Double *mean, Rox_Double *median, Rox_Double *std, Rox_Calibration_Stereo_Perspective_SE3 obj, Rox_Uint id)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Point2D_Double reproj = NULL;
   Rox_Array2D_Double norm = NULL;
   Rox_Double **dn, sum = 0;

   Rox_Uint nbpoints;
   Rox_Double u1, u2, v1, v2;


   if (!min || !max || !mean || !median || !std || !obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (id > obj->lhomographies->used - 1)
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   nbpoints = obj->model->used;

   reproj = (Rox_Point2D_Double )rox_memory_allocate(sizeof(*reproj), nbpoints);
   if(!reproj)
   {
      error = ROX_ERROR_NULL_POINTER;
      ROX_ERROR_CHECK_TERMINATE(error)
   }

   error = rox_array2d_double_new(&norm, nbpoints, 1); ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_get_data_pointer_to_pointer(&dn, norm); ROX_ERROR_CHECK_TERMINATE ( error );

   // Compute 3d-2d projection
   error = rox_point3d_double_transform_project(reproj, obj->lposes->data[id], obj->Kl, obj->model->data, nbpoints);
   ROX_ERROR_CHECK_TERMINATE(error)

   // Compute norm
   for(Rox_Uint j = 0; j < nbpoints; j++)
   {
      // Compute norm
      u1 = reproj[j].u; v1 = reproj[j].v;
      u2 = obj->lpoints->data[id]->data[j].u; v2 = obj->lpoints->data[id]->data[j].v;

      dn[j][0] = sqrt((u1 - u2)*(u1 - u2) + (v1 - v2)*(v1 - v2));
   }

   // make stats
   error = rox_array2d_double_minmax(min, max, norm); ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_median(median, norm);ROX_ERROR_CHECK_TERMINATE(error)

   sum = 0;
   for(Rox_Uint j = 0; j < nbpoints; j++)
   {
      sum += dn[j][0];
   }

   *mean = sum / (Rox_Double)nbpoints;

   sum = 0;
   for(Rox_Uint j = 0; j < nbpoints; j++)
   {
      sum += (dn[j][0] - (*mean)) * (dn[j][0] - (*mean));
   }

   *std = sqrt(sum / (Rox_Double)nbpoints);

   function_terminate:
   rox_memory_delete(reproj);
   rox_array2d_double_del(&norm);

   return error;
}

Rox_ErrorCode rox_calibration_stereo_perspective_se3_get_right_statistics(Rox_Double *min, Rox_Double *max, Rox_Double *mean, Rox_Double *median, Rox_Double *std, Rox_Calibration_Stereo_Perspective_SE3 obj, Rox_Uint id)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Point2D_Double reproj = NULL;
   Rox_Array2D_Double norm = NULL;
   Rox_Array2D_Double pose = NULL;
   Rox_Double **dn, sum = 0;

   Rox_Uint nbpoints;
   Rox_Double u1, u2, v1, v2;

   if(!min || !max || !mean || !median || !std || !obj)
      {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   if(id > obj->rhomographies->used - 1)
      {error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE(error)}

   nbpoints = obj->model->used;

   reproj = (Rox_Point2D_Double ) rox_memory_allocate(sizeof(*reproj), nbpoints);
   if(!reproj)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }


   error = rox_array2d_double_new(&norm, nbpoints, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer_to_pointer(&dn, norm);
   ROX_ERROR_CHECK_TERMINATE ( error );


   error = rox_array2d_double_new(&pose, 4, 4); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Compute right pose
   error = rox_array2d_double_mulmatmat(pose, obj->rTl, obj->lposes->data[id]);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Compute 3d-2d projection
   error = rox_point3d_double_transform_project(reproj, pose, obj->Kr, obj->model->data, nbpoints);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Compute norm
   for(Rox_Uint j = 0; j < nbpoints; j++)
   {
      // Compute norm
      u1 = reproj[j].u; v1 = reproj[j].v;
      u2 = obj->rpoints->data[id]->data[j].u; v2 = obj->rpoints->data[id]->data[j].v;

      dn[j][0] = sqrt((u1 - u2)*(u1 - u2) + (v1 - v2)*(v1 - v2));
   }

   // make stats

   error = rox_array2d_double_minmax(min, max, norm); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_median(median, norm);
   ROX_ERROR_CHECK_TERMINATE ( error );

   sum = 0;
   for(Rox_Uint j = 0; j < nbpoints; j++)
   {
      sum += dn[j][0];
   }

   *mean = sum / (Rox_Double)nbpoints;

   sum = 0;
   for( Rox_Uint j = 0; j < nbpoints; j++)
   {
      sum += (dn[j][0] - (*mean)) * (dn[j][0] - (*mean));
   }

   *std = sqrt(sum / (Rox_Double)nbpoints);

function_terminate:
   rox_memory_delete(reproj);
   rox_array2d_double_del(&norm);
   rox_array2d_double_del(&pose);

   return error;
}
