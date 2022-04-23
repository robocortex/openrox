//==============================================================================
//
//    OPENROX   : File stereo_calibration.c
//
//    Contents  : Implementation of stereo_calibration module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "stereo_calibration.h"
#include "stereo_calibration_struct.h"

#include <stdio.h>

#include <baseproc/maths/maths_macros.h>
#include <baseproc/array/multiply/mulmatmat.h>
#include <baseproc/array/scale/scale.h>
#include <baseproc/array/fill/fillval.h>
#include <baseproc/array/inverse/svdinverse.h>
#include <baseproc/array/solve/svd_solve.h>
#include <baseproc/array/minmax/minmax.h>
#include <baseproc/array/median/median.h>
#include <baseproc/geometry/transforms/transform_tools.h>
#include <baseproc/geometry/point/point2d_projection_from_point3d_transform.h>
#include <baseproc/geometry/point/point2d_projection_from_point3d.h>
#include <baseproc/maths/maths_macros.h>
#include <baseproc/maths/linalg/matse3.h>
#include <baseproc/maths/linalg/generators/algut3.h>

#include <baseproc/calculus/jacobians/jacobian_perspective_stereo_calibration.h>

#include <inout/system/print.h>
#include <inout/system/errors_print.h>

#define threshold_reprojection 1.5

Rox_ErrorCode rox_calibration_stereo_perspective_new(Rox_Calibration_Stereo_Perspective * obj)
{
   Rox_ErrorCode error =  ROX_ERROR_NONE;
   Rox_Calibration_Stereo_Perspective ret = NULL;

   if(!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   *obj = NULL;

   ret = (Rox_Calibration_Stereo_Perspective)rox_memory_allocate(sizeof(*ret), 1);
   if(!ret) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   ret->left = 0;
   ret->right = 0;
   ret->rTl = 0;
   ret->poses = 0;


   error = rox_calibration_mono_perspective_new(&ret->left); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_calibration_mono_perspective_new(&ret->right);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_objset_array2d_double_new(&ret->poses, 10); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&ret->rTl, 4, 4);
   ROX_ERROR_CHECK_TERMINATE ( error );

    *obj = ret;

function_terminate:
    if(error) rox_calibration_stereo_perspective_del(&ret);

    return error;
}

Rox_ErrorCode rox_calibration_stereo_perspective_del(Rox_Calibration_Stereo_Perspective * obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Calibration_Stereo_Perspective todel = NULL;

   if(!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   todel = *obj;
   *obj = NULL;

   if(!todel) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   rox_calibration_mono_perspective_del(&todel->left);
   rox_calibration_mono_perspective_del(&todel->right);
   rox_objset_array2d_double_del(&todel->poses);
   rox_array2d_double_del(&todel->rTl);

   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_calibration_stereo_perspective_set_model_points(Rox_Calibration_Stereo_Perspective obj, Rox_Point3D_Double model, Rox_Uint count)
{
    Rox_ErrorCode error = ROX_ERROR_NONE;

    if(!obj || !model) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

    error = rox_calibration_mono_perspective_set_model_points(obj->left, model, count);
    ROX_ERROR_CHECK_TERMINATE ( error );

    error = rox_calibration_mono_perspective_set_model_points(obj->right, model, count);
    ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_calibration_stereo_perspective_add_current_points(Rox_Calibration_Stereo_Perspective obj, Rox_Point2D_Double  left_pts, Rox_Point2D_Double  right_pts, Rox_Uint count)
{
    Rox_ErrorCode error = ROX_ERROR_NONE;

    if(!obj || !left_pts || !right_pts) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

    error = rox_calibration_mono_perspective_add_current_points(obj->left, left_pts, count);
    ROX_ERROR_CHECK_TERMINATE ( error );

    error = rox_calibration_mono_perspective_add_current_points(obj->right, right_pts, count);
    ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_calibration_stereo_perspective_add_current_homographies(Rox_Calibration_Stereo_Perspective obj, Rox_Array2D_Double Gl, Rox_Array2D_Double Gr)
{
    Rox_ErrorCode error = ROX_ERROR_NONE;

    Rox_Array2D_Double pose = NULL;

    if(!obj || !Gl || !Gr) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}


    error = rox_array2d_double_new(&pose, 4, 4); 
    ROX_ERROR_CHECK_TERMINATE ( error );
    
    error = rox_calibration_mono_perspective_add_homography(obj->left, Gl); 
    ROX_ERROR_CHECK_TERMINATE ( error );
    
    error = rox_calibration_mono_perspective_add_homography(obj->right, Gr); 
    ROX_ERROR_CHECK_TERMINATE ( error );
    
    error = rox_objset_array2d_double_append(obj->poses, pose); 
    ROX_ERROR_CHECK_TERMINATE ( error );


function_terminate:
    if(error) rox_array2d_double_del(&pose);
    return error;
}

Rox_ErrorCode rox_calibration_stereo_perspective_set_intrinsics(Rox_Calibration_Stereo_Perspective obj, Rox_Array2D_Double Kl, Rox_Array2D_Double Kr)
{
    Rox_ErrorCode error = ROX_ERROR_NONE;

    if(!obj || !Kl || !Kr) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}


    error = rox_calibration_mono_perspective_set_intrinsics(obj->left, Kl); 
    ROX_ERROR_CHECK_TERMINATE ( error );
    
    error = rox_calibration_mono_perspective_set_intrinsics(obj->right, Kr); 
    ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_calibration_stereo_perspective_set_resolution(Rox_Calibration_Stereo_Perspective obj, Rox_Sint cols, Rox_Sint rows)
{
    Rox_ErrorCode error = ROX_ERROR_NONE;

    if(!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

    error = rox_array2d_double_set_value(obj->left->K, 0, 2, cols / 2.0); ROX_ERROR_CHECK_TERMINATE ( error );
    error = rox_array2d_double_set_value(obj->left->K, 1, 2, rows / 2.0); ROX_ERROR_CHECK_TERMINATE ( error );

    error = rox_array2d_double_set_value(obj->right->K, 0, 2, cols / 2.0); ROX_ERROR_CHECK_TERMINATE ( error );
    error = rox_array2d_double_set_value(obj->right->K, 0, 2, rows / 2.0); ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_calibration_stereo_perspective_make(Rox_Calibration_Stereo_Perspective obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double mTl = NULL;
   Rox_Double **dKl = NULL, **dT = NULL;
   Rox_Uint recalibrate = 0;

   Rox_Array2D_Double rTl = NULL;
   Rox_Double min, max, mean, med, std;


   if(!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_new(&mTl, 4, 4); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&rTl, 4, 4); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_calibration_mono_perspective_compute_parameters(obj->left, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_calibration_mono_perspective_compute_parameters(obj->right, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Compute rTl pose
   error = rox_calibration_stereo_perspective_compute_intercamera_pose(obj);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Make data copy
   error = rox_calibration_mono_perspective_save_data(obj->left);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_calibration_mono_perspective_save_data(obj->right);
   ROX_ERROR_CHECK_TERMINATE ( error );


   error = rox_array2d_double_copy(rTl, obj->rTl); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Force SE3 and ut3 matrices
   error = rox_array2d_double_get_data_pointer_to_pointer( &dT, obj->rTl); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   dT[3][0] = 0.0; dT[3][1] = 0.0; dT[3][2] = 0.0; dT[3][3] = 1.0;

   error = rox_array2d_double_get_data_pointer_to_pointer(&dKl, obj->left->K); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   dKl[2][0] = 0.0; dKl[2][1] = 0.0; dKl[2][2] = 1.0;

   // refine estimation
   error = rox_calibration_stereo_perspective_process_nolinear_f_cu_cv(obj); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Get stats
   for (Rox_Uint i = 0; i < obj->left->homographies->used; i++)
   {
      if(obj->left->valid_flags->data[i] == 0 || obj->right->valid_flags->data[i] == 0) continue;

      error = rox_calibration_mono_perspective_get_statistics(&min, &max, &mean, &med, &std, obj->left, i);
      ROX_ERROR_CHECK_TERMINATE ( error );

      if(max > threshold_reprojection)
      {
         obj->left->valid_flags->data[i] = 0;
         recalibrate = 1;
      }

      error = rox_calibration_mono_perspective_get_statistics(&min, &max, &mean, &med, &std, obj->right, i);
      ROX_ERROR_CHECK_TERMINATE ( error );

      if(max > threshold_reprojection)
      {
         obj->right->valid_flags->data[i] = 0;
         recalibrate = 1;
      }
   }

   if(recalibrate)
   {
      // Restore data

      error = rox_calibration_mono_perspective_restore_data(obj->left); 
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_calibration_mono_perspective_restore_data(obj->right); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_copy(obj->rTl, rTl); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_calibration_stereo_perspective_process_nolinear_f_cu_cv(obj); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      for (Rox_Uint i = 0; i < obj->left->homographies->used; i++)
      {
         if(obj->left->valid_flags->data[i] == 0 || obj->right->valid_flags->data[i] == 0) continue;

         error = rox_calibration_mono_perspective_get_statistics(&min, &max, &mean, &med, &std, obj->left, i);
         ROX_ERROR_CHECK_TERMINATE ( error );

         if(max > 1.0)
         {
            error = ROX_ERROR_PROCESS_FAILED;
            ROX_ERROR_CHECK_TERMINATE ( error );
         }

         error = rox_calibration_mono_perspective_get_statistics(&min, &max, &mean, &med, &std, obj->right, i);
         ROX_ERROR_CHECK_TERMINATE ( error );

         if(max > 1.0)
         {
            error = ROX_ERROR_PROCESS_FAILED;
            ROX_ERROR_CHECK_TERMINATE(error)
         }
      }
   }

function_terminate:
   rox_array2d_double_del(&mTl);
   rox_array2d_double_del(&rTl);
   return error;
}

Rox_ErrorCode rox_calibration_stereo_perspective_get_results(Rox_Array2D_Double K1, Rox_Array2D_Double K2, Rox_Array2D_Double pose, Rox_Calibration_Stereo_Perspective obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if(!K1 || !K2 || !pose || !obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}


   error = rox_array2d_double_copy(K1, obj->left->K); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_copy(K2, obj->right->K); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_copy(pose, obj->rTl); 
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_calibration_stereo_perspective_process_nolinear(Rox_Calibration_Stereo_Perspective obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint nbimages, nbpoints;
   Rox_Uint Kdll = 5, Tdll = 6, iter = 10;
   Rox_Uint Jcols, Jrows, start_col, start_row;
   Rox_Double lambda = 0.9;
   Rox_Array2D_Double J = 0, JKr = 0, JKl = 0, JTrl = 0, JTl = 0, JTr = 0, rTo = 0;
   Rox_Array2D_Double b = 0, x = 0, xKl = 0, xKr = 0, xTrl = 0, xT =0;
   Rox_Array2D_Double Kl, Kr, lTo, rTl;
   Rox_DynVec_Point3D_Double model;

   Rox_Point2D_Double cur_left = NULL;
   Rox_Point2D_Double cur_right = NULL;
   Rox_Double *zl = 0, *zr = 0;


   if(!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   nbimages = obj->left->poses->used;
   nbpoints = obj->left->model->used;

   // Allocate J
   Jrows = 4*nbpoints * nbimages;
   Jcols = 2*Kdll + Tdll + Tdll * nbimages; // Kl - Kr - Trl - T(1) - ... - T(nbimages)


   error = rox_array2d_double_new(&rTo, 4, 4); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&J, Jrows, Jcols); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&b, 4*nbpoints*nbimages, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&x, Jcols, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   cur_left = (Rox_Point2D_Double) rox_memory_allocate(sizeof(*cur_left), nbpoints);
   if(!cur_left)
   {
      error = ROX_ERROR_NULL_POINTER;
      ROX_ERROR_CHECK_TERMINATE(error)
   }

   cur_right = (Rox_Point2D_Double) rox_memory_allocate(sizeof(*cur_right), nbpoints);
   if(!cur_right)
   {
      error = ROX_ERROR_NULL_POINTER;
      ROX_ERROR_CHECK_TERMINATE(error)
   }

   zl = (Rox_Double*) rox_memory_allocate(sizeof(*zl), nbpoints);
   if(!zl)
   {
      error = ROX_ERROR_NULL_POINTER;
      ROX_ERROR_CHECK_TERMINATE(error)
   }

   zr = (Rox_Double*)rox_memory_allocate(sizeof(*zr), nbpoints);
   if(!zr)
   {
      error = ROX_ERROR_NULL_POINTER;
      ROX_ERROR_CHECK_TERMINATE(error)
   }

   for(Rox_Uint k = 0; k < iter; k++)
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
         Kl = obj->left->K;
         Kr = obj->right->K;
         lTo = obj->left->poses->data[i];
         rTl = obj->rTl;
         model = obj->left->model;

         // valid images
         if(obj->left->valid_flags->data[i] == 0 || obj->right->valid_flags->data[i] == 0) continue;

         // Get Jacobian subviews
         start_col = 0;
         start_row = 4*nbpoints*i;
         error = rox_array2d_double_new_subarray2d(&JKl, J, start_row, start_col, 2*nbpoints, Kdll);
         ROX_ERROR_CHECK_TERMINATE(error)

         start_col = Kdll;
         start_row = 4 * nbpoints * i  + 2 *nbpoints;
         error = rox_array2d_double_new_subarray2d(&JKr, J, start_row, start_col, 2*nbpoints, Kdll);
         ROX_ERROR_CHECK_TERMINATE(error)

         start_col = 2 * Kdll;
         start_row = 4 * nbpoints * i + 2 *nbpoints;
         error = rox_array2d_double_new_subarray2d(&JTrl,J, start_row, start_col, 2*nbpoints, Tdll);
         ROX_ERROR_CHECK_TERMINATE(error)

         start_col = 2 * Kdll + Tdll + i * Tdll;
         start_row = 4 * nbpoints * i;
         error = rox_array2d_double_new_subarray2d(&JTl, J, start_row, start_col, 2*nbpoints, Tdll);
         ROX_ERROR_CHECK_TERMINATE(error)

         start_col = 2 * Kdll + Tdll + i * Tdll;
         start_row = 4 * nbpoints * i + 2 *nbpoints;
         error = rox_array2d_double_new_subarray2d(&JTr, J, start_row, start_col, 2*nbpoints, Tdll);
         ROX_ERROR_CHECK_TERMINATE(error)

         // rTo = rTl * lTo
         error = rox_array2d_double_mulmatmat(rTo, rTl, lTo);
         ROX_ERROR_CHECK_TERMINATE(error)

         error = rox_point2d_double_transform_project(cur_left, zl, Kl, lTo, model->data, nbpoints);
         ROX_ERROR_CHECK_TERMINATE(error)

         error = rox_point2d_double_transform_project(cur_right,zr, Kr, rTo, model->data, nbpoints);
         ROX_ERROR_CHECK_TERMINATE(error)

         // compute jacobians
         error = rox_jacobian_perspective_stereo_calibration(JKl, cur_left, nbpoints);
         ROX_ERROR_CHECK_TERMINATE(error)

         error = rox_jacobian_perspective_stereo_calibration(JKr, cur_right, nbpoints);
         ROX_ERROR_CHECK_TERMINATE(error)

         error = rox_jacobian_perspective_stereo_calibration_pose_intercamera(JTrl, Kr, rTl, lTo, rTo, model->data, nbpoints);
         ROX_ERROR_CHECK_TERMINATE(error)

         error = rox_jacobian_perspective_stereo_calibration_pose(JTl, Kl, lTo, model->data, cur_left, zl, nbpoints);
         ROX_ERROR_CHECK_TERMINATE(error)

         error = rox_jacobian_perspective_stereo_calibration_pose(JTr, Kr, rTo, model->data, cur_right, zr, nbpoints);
         ROX_ERROR_CHECK_TERMINATE(error)

         // Build vector b
         for(Rox_Uint j = 0; j < nbpoints; j++)
         {
               Rox_Uint row = 4 * nbpoints * i + j * 2;

               error = rox_array2d_double_set_value(b, row  , 0, cur_left[j].u  - obj->left->points->data[i]->data[j].u);
               ROX_ERROR_CHECK_TERMINATE(error)

               error = rox_array2d_double_set_value(b, row+1, 0, cur_left[j].v  - obj->left->points->data[i]->data[j].v);
               ROX_ERROR_CHECK_TERMINATE(error)
         }
         for(Rox_Uint j = 0; j < nbpoints; j++)
         {
               Rox_Uint row = 4 * nbpoints * i + 2 * nbpoints + j * 2;

               error = rox_array2d_double_set_value(b, row  , 0, cur_right[j].u - obj->right->points->data[i]->data[j].u);
               ROX_ERROR_CHECK_TERMINATE(error)

               error = rox_array2d_double_set_value(b, row+1, 0, cur_right[j].v - obj->right->points->data[i]->data[j].v);
               ROX_ERROR_CHECK_TERMINATE(error)
         }

         // Free subviews
         rox_array2d_double_del(&JKr);
         rox_array2d_double_del(&JKl);
         rox_array2d_double_del(&JTr);
         rox_array2d_double_del(&JTl);
         rox_array2d_double_del(&JTrl);
      }

      // Solve x = pinv(J)*b

      error = rox_svd_solve(x, J, b); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_scale(x, x, -lambda); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_new(&xKl, 6, 1);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_new(&xKr, 6, 1);
      ROX_ERROR_CHECK_TERMINATE ( error );


      //error = rox_array2d_double_new_subarray2d(&xKl, x, 0, 0, Kdll, 1); 
      //ROX_ERROR_CHECK_TERMINATE ( error );

      //error = rox_array2d_double_new_subarray2d(&xKr, x, Kdll, 0, Kdll, 1); 
      //ROX_ERROR_CHECK_TERMINATE ( error );

      Rox_Double ** x_data = NULL;
      error = rox_array2d_double_get_data_pointer_to_pointer(&x_data, x);
      ROX_ERROR_CHECK_TERMINATE ( error );

      Rox_Double ** xKl_data = NULL;
      error = rox_array2d_double_get_data_pointer_to_pointer(&xKl_data, xKl);
      ROX_ERROR_CHECK_TERMINATE ( error );

      xKl_data[0][0] = x_data[0][0];
      xKl_data[1][0] = x_data[1][0];
      xKl_data[2][0] = x_data[2][0];
      xKl_data[3][0] = x_data[3][0];
      xKl_data[4][0] = x_data[4][0];
      xKl_data[5][0] = 0.0;

      Rox_Double ** xKr_data = NULL;
      error = rox_array2d_double_get_data_pointer_to_pointer(&xKr_data, xKr);

      xKr_data[0][0] = x_data[5][0];
      xKr_data[1][0] = x_data[6][0];
      xKr_data[2][0] = x_data[7][0];
      xKr_data[3][0] = x_data[8][0];
      xKr_data[4][0] = x_data[9][0];
      xKr_data[5][0] = 0.0;


      error = rox_array2d_double_new_subarray2d(&xTrl , x, 2*Kdll, 0, Tdll, 1); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Update K and T
      error = rox_array2d_double_scale_inplace(xKl, -1.0);
      ROX_ERROR_CHECK_TERMINATE( error );

      
      error = rox_matut3_update_left(obj->left->K, xKl); 
      ROX_ERROR_CHECK_TERMINATE ( error );
   
      error = rox_array2d_double_scale_inplace(xKr, -1.0);
      ROX_ERROR_CHECK_TERMINATE( error );

      error = rox_matut3_update_left(obj->right->K, xKr); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_matse3_update_right(obj->rTl, xTrl); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      for(Rox_Uint i = 0; i < nbimages; i++)
      {
         // valid images
         if(obj->left->valid_flags->data[i] == 0 || obj->right->valid_flags->data[i] == 0) continue;


         error = rox_array2d_double_new_subarray2d(&xT, x, 2*Kdll + Tdll, 0, Tdll, 1); 
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_matse3_update_right(obj->left->poses->data[i], xT); 
         ROX_ERROR_CHECK_TERMINATE ( error );

         rox_array2d_double_del(&xT);
      }
   }

   // Update the right poses after the visual servoing
   for (Rox_Uint i = 0; i < nbimages; i++)
   {
      // valid images
      if(obj->left->valid_flags->data[i] == 0 || obj->right->valid_flags->data[i] == 0) continue;

      error = rox_array2d_double_mulmatmat(obj->right->poses->data[i], obj->rTl, obj->left->poses->data[i]);
      ROX_ERROR_CHECK_TERMINATE(error)
   }

function_terminate:
   rox_array2d_double_del(&JKr);
   rox_array2d_double_del(&JKl);
   rox_array2d_double_del(&JTr);
   rox_array2d_double_del(&JTl);
   rox_array2d_double_del(&JTrl);
   rox_array2d_double_del(&J);
   rox_array2d_double_del(&xT);
   rox_array2d_double_del(&b);
   rox_array2d_double_del(&x);
   rox_array2d_double_del(&xKl);
   rox_array2d_double_del(&xKr);
   rox_array2d_double_del(&xTrl);
   rox_array2d_double_del(&xT);

   rox_memory_delete(cur_left);
   rox_memory_delete(cur_right);
   rox_memory_delete(zl);
   rox_memory_delete(zr);

   return error;
}

Rox_ErrorCode rox_calibration_stereo_perspective_process_nolinear_f_cu_cv(Rox_Calibration_Stereo_Perspective obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint nbimages, nbpoints;
   Rox_Uint Kdll = 3, Tdll = 6, iter = 10;
   Rox_Uint Jcols, Jrows, start_col, start_row;
   Rox_Double lambda = 0.9;
   Rox_Array2D_Double J = 0, JKr = 0, JKl = 0, JTrl = 0, JTl = 0, JTr = 0, rTo = 0;
   Rox_Array2D_Double b = 0, x = 0, xKl = 0, xKr = 0, xTrl = 0, xT =0, xK =0;
   Rox_Array2D_Double Kl, Kr, lTo, rTl;
   Rox_DynVec_Point3D_Double model;

   Rox_Point2D_Double cur_left = NULL;
   Rox_Point2D_Double cur_right = NULL;
   Rox_Double *zl = 0, *zr = 0;
   Rox_Double **dxK, **dxKl, **dxKr;

   if(!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   nbimages = obj->left->poses->used;
   nbpoints = obj->left->model->used;

   // Allocate J
   Jrows = 4*nbpoints * nbimages;
   Jcols = 2*Kdll + Tdll + Tdll * nbimages; // Kl - Kr - Trl - T(1) - ... - T(nbimages)


   error = rox_array2d_double_new(&rTo, 4, 4); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&J, Jrows, Jcols); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&b, 4*nbpoints*nbimages, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&x, Jcols, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Allocate the vector for the algut3
   error = rox_array2d_double_new(&xK, 6, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   cur_left  =(Rox_Point2D_Double ) rox_memory_allocate(sizeof(*cur_left), nbpoints);
   if (!cur_left)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   cur_right = (Rox_Point2D_Double ) rox_memory_allocate(sizeof(*cur_right), nbpoints);
   if (!cur_right)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   zl = (Rox_Double*) rox_memory_allocate(sizeof(*zl), nbpoints);
   if (!zl)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   zr = (Rox_Double*) rox_memory_allocate(sizeof(*zr), nbpoints);
   if (!zl)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_get_data_pointer_to_pointer(&dxK, xK);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for(Rox_Uint k = 0; k < iter; k++)
   {

      // Reset J

      error = rox_array2d_double_fillval(J, 0.0); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_fillval(x, 0.0); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_fillval(b, 0.0); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      for (Rox_Uint i = 0; i < nbimages; i++)
      {
         Kl = obj->left->K;
         Kr = obj->right->K;
         lTo = obj->left->poses->data[i];
         rTl = obj->rTl;
         model = obj->left->model;

         // valid images
         if(obj->left->valid_flags->data[i] == 0 || obj->right->valid_flags->data[i] == 0) continue;

         // Get Jacobian subviews
         start_col = 0;
         start_row = 4*nbpoints*i;
         error = rox_array2d_double_new_subarray2d(&JKl, J, start_row, start_col, 2*nbpoints, Kdll);
         ROX_ERROR_CHECK_TERMINATE ( error );

         start_col = Kdll;
         start_row = 4 * nbpoints * i  + 2 *nbpoints;
         error = rox_array2d_double_new_subarray2d(&JKr, J, start_row, start_col, 2*nbpoints, Kdll);
         ROX_ERROR_CHECK_TERMINATE ( error );

         start_col = 2 * Kdll;
         start_row = 4 * nbpoints * i + 2 *nbpoints;
         error = rox_array2d_double_new_subarray2d(&JTrl,J, start_row, start_col, 2*nbpoints, Tdll);
         ROX_ERROR_CHECK_TERMINATE ( error );

         start_col = 2 * Kdll + Tdll + i * Tdll;
         start_row = 4 * nbpoints * i;
         error = rox_array2d_double_new_subarray2d(&JTl, J, start_row, start_col, 2*nbpoints, Tdll);
         ROX_ERROR_CHECK_TERMINATE ( error );

         start_col = 2 * Kdll + Tdll + i * Tdll;
         start_row = 4 * nbpoints * i + 2 *nbpoints;
         error = rox_array2d_double_new_subarray2d(&JTr, J, start_row, start_col, 2*nbpoints, Tdll);
         ROX_ERROR_CHECK_TERMINATE ( error );

         // rTo = rTl * lTo
         error = rox_array2d_double_mulmatmat(rTo, rTl, lTo); ROX_ERROR_CHECK_TERMINATE(error)


         error = rox_point2d_double_transform_project(cur_left, zl, Kl, lTo, model->data, nbpoints); 
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_point2d_double_transform_project(cur_right,zr, Kr, rTo, model->data, nbpoints); 
         ROX_ERROR_CHECK_TERMINATE ( error );

         // compute jacobians
         error = rox_jacobian_perspective_stereo_calibration_f_cu_cv(JKl, cur_left, nbpoints); 
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_jacobian_perspective_stereo_calibration_f_cu_cv(JKr, cur_right, nbpoints);
         ROX_ERROR_CHECK_TERMINATE ( error );


         error = rox_jacobian_perspective_stereo_calibration_pose_intercamera(JTrl, Kr, rTl, lTo, rTo, model->data, nbpoints); 
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_jacobian_perspective_stereo_calibration_pose(JTl, Kl, lTo, model->data, cur_left, zl, nbpoints); 
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_jacobian_perspective_stereo_calibration_pose(JTr, Kr, rTo, model->data, cur_right, zr, nbpoints); 
         ROX_ERROR_CHECK_TERMINATE ( error );

         // Build vector b
         for(Rox_Uint j = 0; j < nbpoints; j++)
         {
               Rox_Uint row = 4 * nbpoints * i + j * 2;

               error = rox_array2d_double_set_value(b, row  , 0, cur_left[j].u  - obj->left->points->data[i]->data[j].u);
               ROX_ERROR_CHECK_TERMINATE(error)
               error = rox_array2d_double_set_value(b, row+1, 0, cur_left[j].v  - obj->left->points->data[i]->data[j].v);
               ROX_ERROR_CHECK_TERMINATE(error)
         }

         for(Rox_Uint j = 0; j < nbpoints; j++)
         {
               Rox_Uint row = 4 * nbpoints * i + 2 * nbpoints + j * 2;

               error = rox_array2d_double_set_value(b, row  , 0, cur_right[j].u - obj->right->points->data[i]->data[j].u);
               error = rox_array2d_double_set_value(b, row+1, 0, cur_right[j].v - obj->right->points->data[i]->data[j].v);
         }

         // Free subviews
         rox_array2d_double_del(&JKr);
         rox_array2d_double_del(&JKl);
         rox_array2d_double_del(&JTr);
         rox_array2d_double_del(&JTl);
         rox_array2d_double_del(&JTrl);
      }

      // Solve x = pinv(J)*b

      error = rox_svd_solve(x, J, b); 
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_array2d_double_scale(x, x, -lambda); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_new_subarray2d(&xKl, x, 0, 0, Kdll, 1); 
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_array2d_double_new_subarray2d(&xKr, x, Kdll, 0, Kdll, 1); 
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_array2d_double_new_subarray2d(&xTrl , x, 2*Kdll, 0, Tdll, 1); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Update K and T

      // Build xK
      error = rox_array2d_double_get_data_pointer_to_pointer(&dxKl, xKl);
      ROX_ERROR_CHECK_TERMINATE ( error );

      dxK[0][0] = dxKl[0][0]; // f
      dxK[1][0] = 0.0;        // skew
      dxK[2][0] = dxKl[1][0]; // cu
      dxK[3][0] = dxKl[0][0]; // f
      dxK[4][0] = dxKl[2][0]; // cv
      dxK[5][0] = 0.0;

      error = rox_array2d_double_scale_inplace(xK, -1.0);
      ROX_ERROR_CHECK_TERMINATE( error );


      error = rox_matut3_update_left(obj->left->K, xK); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Build xK
      error = rox_array2d_double_get_data_pointer_to_pointer(&dxKr, xKr);
      ROX_ERROR_CHECK_TERMINATE ( error );

      dxK[0][0] = dxKr[0][0]; // f
      dxK[1][0] = 0.0;        // skew
      dxK[2][0] = dxKr[1][0]; // cu
      dxK[3][0] = dxKr[0][0]; // f
      dxK[4][0] = dxKr[2][0]; // cv
      dxK[5][0] = 0.0;

      error = rox_array2d_double_scale_inplace(xK, -1.0);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_matut3_update_left(obj->right->K, xK);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_matse3_update_right(obj->rTl, xTrl);
      ROX_ERROR_CHECK_TERMINATE ( error );

      for (Rox_Uint i = 0; i < nbimages; i++)
      {
         // valid images
         if(obj->left->valid_flags->data[i] == 0 || obj->right->valid_flags->data[i] == 0) continue;

         error = rox_array2d_double_new_subarray2d(&xT, x, 2*Kdll + Tdll + Tdll*i, 0, Tdll, 1);
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_matse3_update_right(obj->left->poses->data[i], xT);
         ROX_ERROR_CHECK_TERMINATE ( error );
         rox_array2d_double_del ( &xT );
      }
   }

   // Update the right poses after the visual servoing
   for (Rox_Uint i = 0; i < nbimages; i++)
   {
      // valid images
      if(obj->left->valid_flags->data[i] == 0 || obj->right->valid_flags->data[i] == 0) continue;

      error = rox_array2d_double_mulmatmat(obj->right->poses->data[i], obj->rTl, obj->left->poses->data[i]);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   rox_array2d_double_del(&JKr);
   rox_array2d_double_del(&JKl);
   rox_array2d_double_del(&JTr);
   rox_array2d_double_del(&JTl);
   rox_array2d_double_del(&JTrl);
   rox_array2d_double_del(&J);
   rox_array2d_double_del(&xT);
   rox_array2d_double_del(&b);
   rox_array2d_double_del(&x);
   rox_array2d_double_del(&xK);
   rox_array2d_double_del(&xKl);
   rox_array2d_double_del(&xKr);
   rox_array2d_double_del(&xTrl);
   rox_array2d_double_del(&xT);

   rox_memory_delete(cur_left);
   rox_memory_delete(cur_right);
   rox_memory_delete(zl);
   rox_memory_delete(zr);

   return error;
}

Rox_ErrorCode rox_calibration_stereo_perspective_print_statistics(Rox_Calibration_Stereo_Perspective obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double norm = 0;
   Rox_Uint nbimages, nbpoints;

   Rox_Point2D_Double reproj = 0;
   Rox_Double **dn;


   if ( !obj ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   nbimages = obj->left->poses->used;
   nbpoints = obj->left->model->used;

   reproj = (Rox_Point2D_Double ) rox_memory_allocate(sizeof(*reproj), nbpoints);
   if (!reproj)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }


   error = rox_array2d_double_new(&norm, nbpoints, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer_to_pointer(&dn, norm);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Right
   rox_log("Right \n");
   for (Rox_Uint i = 0; i < nbimages; i++)
   {
      Rox_Double min, max, mean, sum, var, med, u1, v1, u2, v2;

      // valid images
      if(obj->left->valid_flags->data[i] == 0 || obj->right->valid_flags->data[i] == 0) continue;

      error = rox_point3d_double_transform_project(reproj, obj->right->poses->data[i], obj->right->K, obj->right->model->data, nbpoints);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Compute norm
      for(Rox_Uint j = 0; j < nbpoints; j++)
      {
         // Compute norm
         u1 = reproj[j].u; v1 = reproj[j].v;
         u2 = obj->right->points->data[i]->data[j].u; v2 = obj->right->points->data[i]->data[j].v;

         dn[j][0] = sqrt((u1 - u2)*(u1 - u2) + (v1 - v2)*(v1 - v2));
      }

      // make stats
      error = rox_array2d_double_minmax ( &min, &max, norm );
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_median ( &med, norm );
      ROX_ERROR_CHECK_TERMINATE ( error );

      sum = 0;
      for(Rox_Uint j = 0; j < nbpoints; j++)
      {
         sum += dn[j][0];
      }

      mean = sum / (Rox_Double) nbpoints;

      sum = 0;
      for(Rox_Uint j = 0; j < nbpoints; j++)
      {
         sum += (dn[j][0] - mean) * (dn[j][0] - mean);
      }
      var = sqrt(sum / (Rox_Double)nbpoints);

      rox_log("min %f, max %f, mean %f, variance %f, median %f\n", min, max, mean, var, med);
   }

   // Left
   rox_log("Left \n");
   for (Rox_Uint i = 0; i < nbimages; i++)
   {
      Rox_Double min, max, mean, sum, var, med, u1, v1, u2, v2;

      // valid images
      if(obj->left->valid_flags->data[i] == 0 || obj->right->valid_flags->data[i] == 0) continue;

      error = rox_point3d_double_transform_project(reproj, obj->left->poses->data[i], obj->left->K, obj->left->model->data, nbpoints);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Compute norm
      for(Rox_Uint j = 0; j < nbpoints; j++)
      {
         // Compute norm
         u1 = reproj[j].u; v1 = reproj[j].v;
         u2 = obj->left->points->data[i]->data[j].u; v2 = obj->left->points->data[i]->data[j].v;

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
      var = sqrt(sum / (Rox_Double)nbpoints);

      rox_log("min %f, max %f, mean %f, variance %f, median %f\n", min, max, mean, var, med);
   }

   function_terminate:
   rox_memory_delete(reproj);
   rox_array2d_double_del(&norm);

   return error;
}

Rox_ErrorCode rox_calibration_stereo_perspective_compute_intercamera_pose(Rox_Calibration_Stereo_Perspective obj)
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

   Rox_Uint j, nbpose;


   if(!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   nbpose = 0;

   for (Rox_Uint i = 0; i < obj->poses->used; i++)
   {
      // valid images
      if(obj->left->valid_flags->data[i] == 1 && obj->right->valid_flags->data[i] == 1)
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

   j = 0;

   // Build all rTl poses
   for (Rox_Uint i = 0; i < obj->poses->used; i++)
   {
      // valid images
      if(obj->left->valid_flags->data[i] == 0 || obj->right->valid_flags->data[i] == 0) continue;

      error = rox_array2d_double_svdinverse(mTl, obj->left->poses->data[i]); ROX_ERROR_CHECK_TERMINATE ( error );
      error = rox_array2d_double_mulmatmat(obj->poses->data[i], obj->right->poses->data[i], mTl); ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_new_subarray2d(&R, obj->poses->data[i], 0, 0, 3, 3); ROX_ERROR_CHECK_TERMINATE ( error );
      error = rox_array2d_double_new_subarray2d(&t, obj->poses->data[i], 0, 3, 3, 1); ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_get_data_pointer_to_pointer(&dt, t); ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_transformtools_axisangle_from_rotationmatrix(&ax, &ay, &az, &ang, R); ROX_ERROR_CHECK_TERMINATE ( error );

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
   error = rox_array2d_double_median(&med_ax, rx); ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_median(&med_ay, ry); ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_median(&med_az, rz); ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_median(&med_tx, tx); ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_median(&med_ty, ty); ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_median(&med_tz, tz); ROX_ERROR_CHECK_TERMINATE ( error );

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
   for (Rox_Uint i = 0; i < obj->poses->used; i++)
   {
      Rox_Double diftx, difty, diftz, difrx, difry, difrz;

      // valid images
      if(obj->left->valid_flags->data[i] == 0 || obj->right->valid_flags->data[i] == 0) continue;

      // Translation
      error = rox_array2d_double_new_subarray2d(&R, obj->poses->data[i], 0, 0, 3, 3); ROX_ERROR_CHECK_TERMINATE ( error );
      error = rox_array2d_double_new_subarray2d(&t, obj->poses->data[i], 0, 3, 3, 1); ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_get_data_pointer_to_pointer( &dt, t); ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_transformtools_axisangle_from_rotationmatrix(&ax, &ay, &az, &ang, R); ROX_ERROR_CHECK_TERMINATE ( error );

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
   for(j = 0; j < nbpose; j++)
   {
      sum_nr += (dnr[j][0] - mean_nr) * (dnr[j][0] - mean_nr);
      sum_nt += (dnt[j][0] - mean_nt) * (dnt[j][0] - mean_nt);
   }
   var_nr = sqrt(sum_nr / (Rox_Double)(nbpose-1));
   var_nt = sqrt(sum_nt / (Rox_Double)(nbpose-1));

   // Check images
   j = 0;
   for (Rox_Uint i = 0; i < obj->poses->used; i++)
   {
      // valid images
      if(obj->left->valid_flags->data[i] == 0 || obj->right->valid_flags->data[i] == 0) continue;

      if(dnt[j][0] > 3.0 * var_nt)
      {
         obj->left->valid_flags->data[i] = 0;
         obj->right->valid_flags->data[i] = 0;
         rox_log("Removed %d image\n", i);
      }

      if(dnr[j][0] > 3.0 * var_nr)
      {
         obj->left->valid_flags->data[i] = 0;
         obj->right->valid_flags->data[i] = 0;
         rox_log("Removed %d image\n", i);
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
