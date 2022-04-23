//==============================================================================
//
//    OPENROX   : File bundle.c
//
//    Contents  : Implementation of bundle module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "bundle.h"
#include "bundle_struct.h"
#include "bundle_point_struct.h"
#include "bundle_frame_struct.h"

#include <float.h>

#include <baseproc/maths/maths_macros.h>
#include <baseproc/array/fill/fillunit.h>
#include <baseproc/array/fill/fillval.h>
#include <baseproc/array/decomposition/cholesky.h>
#include <baseproc/array/inverse/lotinverse.h>
#include <baseproc/array/inverse/svdinverse.h>
#include <baseproc/array/multiply/mulmatmat.h>
#include <baseproc/array/multiply/mulmatmattrans.h>
#include <baseproc/array/multiply/mulmattransmat.h>
#include <baseproc/array/substract/substract.h>
#include <baseproc/array/scale/scale.h>
#include <baseproc/array/robust/tukey.h>
#include <baseproc/maths/maths_macros.h>
#include <baseproc/maths/linalg/matse3.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_bundle_new(Rox_Bundle * obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Bundle ret = NULL;

   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *obj = NULL;

   ret = (Rox_Bundle) rox_memory_allocate(sizeof(*ret), 1);
   if (!ret) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret->cameras = NULL;
   ret->frames = NULL;
   ret->measures = NULL;
   ret->points = NULL;

   error = rox_objset_bundle_camera_new(&ret->cameras, 5);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_objset_bundle_frame_new(&ret->frames, 5);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_objset_bundle_point_new(&ret->points, 10);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_objset_bundle_measure_new(&ret->measures, 10);
   ROX_ERROR_CHECK_TERMINATE ( error );

   ret->lambda_multiplier = 2;
   ret->lambda = 1e-12;
   ret->tau = 1e-3;
   ret->max_iterations = 50;
   ret->eps1 = 1e-12;
   ret->eps2 = 1e-12;

   *obj = ret;

function_terminate:
   if (error) rox_bundle_del(&ret);

   return error;
}

Rox_ErrorCode rox_bundle_del(Rox_Bundle * obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Bundle todel = NULL;

   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *obj;
   *obj = NULL;

   if (!todel) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_objset_bundle_frame_del(&todel->frames);
   rox_objset_bundle_camera_del(&todel->cameras);
   rox_objset_bundle_point_del(&todel->points);
   rox_objset_bundle_measure_del(&todel->measures);

   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_bundle_add_camera(Rox_Uint * inserted_idx, Rox_Bundle obj, Rox_Array2D_Double relative, Rox_Array2D_Double calib)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Bundle_Camera camera = NULL;

   if (!obj || !relative || !calib) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_check_size(relative, 4, 4);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(calib, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Copy parameters to camera
   error = rox_bundle_camera_new(&camera);
   ROX_ERROR_CHECK_TERMINATE ( error );
 
   error = rox_array2d_double_copy(camera->calib, calib);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_copy(camera->relative_pose, relative);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_objset_bundle_camera_append(obj->cameras, camera);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // This is needed to avoid to delete the memory, the camera is appended to obj->cameras norw
   camera = NULL;

   // Return back the insertion index
   *inserted_idx = obj->cameras->used - 1;

function_terminate:
   rox_bundle_camera_del(&camera);
   return error;
}

Rox_ErrorCode rox_bundle_add_frame(Rox_Uint * inserted_idx, Rox_Bundle obj, Rox_Array2D_Double pose, Rox_Uint is_fixed)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Bundle_Frame frame = NULL;

   if (!obj || !pose) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_check_size(pose, 4, 4);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_bundle_frame_new(&frame);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_copy(frame->pose, pose);
   ROX_ERROR_CHECK_TERMINATE ( error );
  
   error = rox_objset_bundle_frame_append(obj->frames, frame);
   ROX_ERROR_CHECK_TERMINATE ( error );

   frame->is_fixed = is_fixed;
   frame = NULL;

   *inserted_idx = obj->frames->used - 1;

function_terminate:
   rox_bundle_frame_del(&frame);
   return error;
}

Rox_ErrorCode rox_bundle_add_point (
   Rox_Uint * inserted_idx,
   Rox_Bundle obj,
   Rox_Point3D_Double coords)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Bundle_Point point = NULL;

   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_bundle_point_new(&point);
   ROX_ERROR_CHECK_TERMINATE ( error );
  
   error = rox_objset_bundle_point_append(obj->points, point);
   ROX_ERROR_CHECK_TERMINATE ( error );
  
   point->coords = *coords;
   point = NULL;

   *inserted_idx = obj->points->used - 1;

function_terminate:
   rox_bundle_point_del(&point);
   return error;
}

Rox_ErrorCode rox_bundle_add_measurement (
   Rox_Uint * inserted_idx, Rox_Bundle obj,
   Rox_Point2D_Double coords, Rox_Uint idx_camera, Rox_Uint idx_frame, Rox_Uint idx_point)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Bundle_Measure mes = NULL;

   if (!obj)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Check input indices
  
   if (idx_camera >= obj->cameras->used) 
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (idx_frame >= obj->frames->used) 
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (idx_point >= obj->points->used) 
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_bundle_measure_new(&mes);
   ROX_ERROR_CHECK_TERMINATE ( error );

   mes->frame = obj->frames->data[idx_frame];
   mes->camera = obj->cameras->data[idx_camera];
   mes->point = obj->points->data[idx_point];

   Rox_Double ** dk = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dk, mes->camera->calib);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ifu = 1.0 / dk[0][0];
   Rox_Double ifv = 1.0 / dk[1][1];
   Rox_Double icu = - dk[0][2] * ifu;
   Rox_Double icv = - dk[1][2] * ifv;

   mes->coords_pixels = *coords;
   mes->coords_meters.u = ifu * coords->u + icu;
   mes->coords_meters.v = ifv * coords->v + icv;

   error = rox_dynvec_bundle_measure_append(mes->frame->measures, &mes);
   error = rox_dynvec_bundle_measure_append(mes->point->measures, &mes);

   // To be done at the end of the function (we don't wan to delete mes if added to the set) !!
   error = rox_objset_bundle_measure_append(obj->measures, mes);
   mes = NULL;

   *inserted_idx = obj->measures->used - 1;

function_terminate:
   rox_bundle_measure_del(&mes);
   return error;
}

Rox_ErrorCode rox_bundle_compute_prediction(Rox_Bundle obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   for (Rox_Uint idmes = 0; idmes < obj->measures->used; idmes++)
   {
      obj->measures->data[idmes]->is_invalid = 1;

      error = rox_bundle_measure_predict(obj->measures->data[idmes]);
      ROX_ERROR_CHECK_TERMINATE ( error );

      obj->measures->data[idmes]->is_invalid = 0;
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_bundle_compute_weights(Rox_Bundle obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double dx, dy;

   Rox_Array2D_Double weights = NULL, dist = NULL, wb1 = NULL, wb2 = NULL;

   if (!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   // Estimate the number of measures to create the buffers

   Rox_Double sum = 0.0;
   Rox_Uint nbmes = 0;
   for (Rox_Uint idmes = 0; idmes < obj->measures->used; idmes++)
   {
      if (obj->measures->data[idmes]->is_invalid)
      {
         continue;
      }

      nbmes++;
   }

   // Create buffers

   error = rox_array2d_double_new(&weights, nbmes, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&dist, nbmes, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&wb1, nbmes, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&wb2, nbmes, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double * dd = NULL;
   error = rox_array2d_double_get_data_pointer( &dd, dist);
   Rox_Double * dw = NULL;
   error = rox_array2d_double_get_data_pointer( &dw, weights);

   // Estimate a vector of L2-distances between measures and model

   nbmes = 0;
   for (Rox_Uint idmes = 0; idmes < obj->measures->used; idmes++)
   {
      if (obj->measures->data[idmes]->is_invalid)
      {
         continue;
      }
      dd[nbmes] = sqrt(obj->measures->data[idmes]->distance_pixels);
      nbmes++;
   }

   // Estimate weights

   // CHECK_ERROR_TERMINATE(rox_array2d_double_tukey(weights, wb1, wb2, dist));
   // CHECK_ERROR_TERMINATE(rox_array2d_double_hubers(weights, wb1, wb2, dist));
   error = rox_array2d_double_tukey_bounded(weights, wb1, wb2, dist, 2.0, DBL_MAX);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Assign weights to measures

   nbmes = 0;
   for (Rox_Uint idmes = 0; idmes < obj->measures->used; idmes++)
   {
      if (obj->measures->data[idmes]->is_invalid)
      {
         continue;
      }

      if (dw[nbmes] < DBL_EPSILON)
      {
         obj->measures->data[idmes]->is_invalid = 1;
      }

      obj->measures->data[idmes]->weight = dw[nbmes];
      dx = dw[nbmes] * obj->measures->data[idmes]->error_pixels.u;
      dy = dw[nbmes] * obj->measures->data[idmes]->error_pixels.v;
      sum += dx*dx + dy*dy;
      nbmes++;
   }

   obj->norm_error = sum;

function_terminate:
   rox_array2d_double_del(&weights);
   rox_array2d_double_del(&dist);
   rox_array2d_double_del(&wb1);
   rox_array2d_double_del(&wb2);
   return error;
}

Rox_ErrorCode rox_bundle_compute_hessians(Rox_Bundle obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   for (Rox_Uint idmes = 0; idmes < obj->measures->used; idmes++)
   {
      error = rox_bundle_measure_build_jacobians(obj->measures->data[idmes]);
      if (error) continue;
   }

   for (Rox_Uint idpt = 0; idpt < obj->points->used; idpt++)
   {
      error = rox_bundle_point_compute_hessian(obj->points->data[idpt]);
      if (error) continue;
   }

   for (Rox_Uint idframe = 0; idframe < obj->frames->used; idframe++)
   {
      error = rox_bundle_frame_compute_hessian(obj->frames->data[idframe]);
      if (error) continue;
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_bundle_compute_initial_lambda(Rox_Bundle obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   // max = MAX(H[i,i])

   Rox_Double max = - DBL_MAX;

   for (Rox_Uint idpt = 0; idpt < obj->points->used; idpt++)
   {
      if (obj->points->data[idpt]->is_invalid) continue;

      Rox_Double ** dh = NULL;
      error = rox_array2d_double_get_data_pointer_to_pointer( &dh, obj->points->data[idpt]->hessian);

      if (dh[0][0] > max) max = dh[0][0];
      if (dh[1][1] > max) max = dh[1][1];
      if (dh[2][2] > max) max = dh[2][2];
   }

   for (Rox_Uint idframe = 0; idframe < obj->frames->used; idframe++)
   {
      if (obj->frames->data[idframe]->is_invalid) continue;
      if (obj->frames->data[idframe]->is_fixed) continue;

      Rox_Double ** dh = NULL;
      error = rox_array2d_double_get_data_pointer_to_pointer( &dh, obj->frames->data[idframe]->hessian);

      if (dh[0][0] > max) max = dh[0][0];
      if (dh[1][1] > max) max = dh[1][1];
      if (dh[2][2] > max) max = dh[2][2];
      if (dh[3][3] > max) max = dh[3][3];
      if (dh[4][4] > max) max = dh[4][4];
      if (dh[5][5] > max) max = dh[5][5];
   }

   obj->lambda = max * obj->tau;
   obj->lambda_multiplier = 2;

function_terminate:
   return error;
}

Rox_ErrorCode rox_bundle_check_validity(Rox_Bundle obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint count;
   Rox_Uint something_changed = 1;

   if (!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   // We assume prediction on measure has been done previously, and invalid measures are flagged

   for (Rox_Uint idpt = 0; idpt < obj->points->used; idpt++)
   {
      obj->points->data[idpt]->is_invalid = 0;
   }

   for (Rox_Uint idframe = 0; idframe < obj->frames->used; idframe++)
   {
      obj->frames->data[idframe]->is_invalid = 0;
   }

   while (something_changed)
   {
      something_changed = 0;

      for (Rox_Uint idmes = 0; idmes < obj->measures->used; idmes++)
      {
         if (obj->measures->data[idmes]->is_invalid) continue;

         if (obj->measures->data[idmes]->point->is_invalid || obj->measures->data[idmes]->frame->is_invalid)
         {
            obj->measures->data[idmes]->is_invalid = 1;
            something_changed = 1;
         }
      }

      for (Rox_Uint idpt = 0; idpt < obj->points->used; idpt++)
      {
         Rox_Bundle_Point pt = obj->points->data[idpt];

         if (pt->is_invalid) continue;

         count = 0;
         for (Rox_Uint idmes = 0; idmes < pt->measures->used; idmes++)
         {
            if (pt->measures->data[idmes]->is_invalid) continue;

            count++;
         }

         if (count < 2)
         {
            something_changed = 1;
            pt->is_invalid = 1;
         }
      }

      for (Rox_Uint idframe = 0; idframe < obj->frames->used; idframe++)
      {
         Rox_Bundle_Frame frame = obj->frames->data[idframe];

         if (frame->is_invalid) continue;
         if (frame->is_fixed) continue;

         count = 0;
         for (Rox_Uint idmes = 0; idmes < frame->measures->used; idmes++)
         {
            if (frame->measures->data[idmes]->is_invalid) continue;

            count++;
         }

         if (count < 6)
         {
            something_changed = 1;
            frame->is_invalid = 1;
         }
      }
   }


   // Recompute positions and count of points in valid list
   obj->count_valid_points = 0;
   for (Rox_Uint idpt = 0; idpt < obj->points->used; idpt++)
   {
      if (obj->points->data[idpt]->is_invalid) continue;

      obj->points->data[idpt]->pos_jacobian = obj->count_valid_points;
      obj->count_valid_points++;
   }

   // Recompute positions and count of frame in valid list
   obj->count_valid_frames = 0;
   for (Rox_Uint idframe = 0; idframe < obj->frames->used; idframe++)
   {
      if (obj->frames->data[idframe]->is_invalid) continue;

      obj->frames->data[idframe]->pos_jacobian = obj->count_valid_frames;
      obj->count_valid_frames++;
   }

   // Recompute valid measures counter
   obj->count_valid_measures = 0;
   for (Rox_Uint idmes = 0; idmes < obj->measures->used; idmes++)
   {
      if (obj->measures->data[idmes]->is_invalid) continue;
      obj->count_valid_measures++;
   }

function_terminate:
   return error;
}


Rox_ErrorCode rox_bundle_solve_system(Rox_Bundle obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint idframe, idpoint, idmes, idmes2, i, j, k, poscam, poscam2;

   Rox_Array2D_Double A = NULL;
   Rox_Array2D_Double A_inverse = NULL;
   Rox_Array2D_Double A_lot = NULL;
   Rox_Array2D_Double B = NULL;
   Rox_Array2D_Double Hpp = NULL;
   Rox_Array2D_Double Hpp_chol = NULL;
   Rox_Array2D_Double Hpp_inverse = NULL;
   Rox_Array2D_Double Hpctp = NULL;
   Rox_Array2D_Double xcam = NULL;
   Rox_Array2D_Double xcam_buffer = NULL;
   Rox_Array2D_Double tmp3 = NULL;
   Rox_Double ** da = NULL, *db = NULL, **dhpp = NULL, *dhpctp = NULL, *rowa = NULL;

   Rox_Uint  sizecols_cam;

   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (obj->count_valid_points < 1) 
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   sizecols_cam = obj->count_valid_frames * 6;

   // Buffer initialization
   CHECK_ERROR_TERMINATE(rox_array2d_double_new(&A, sizecols_cam, sizecols_cam));
   CHECK_ERROR_TERMINATE(rox_array2d_double_new(&A_inverse, sizecols_cam, sizecols_cam));
   CHECK_ERROR_TERMINATE(rox_array2d_double_new(&A_lot, sizecols_cam, sizecols_cam));
   CHECK_ERROR_TERMINATE(rox_array2d_double_new(&B, sizecols_cam, 1));
   CHECK_ERROR_TERMINATE(rox_array2d_double_new(&Hpp_chol, 3, 3));
   CHECK_ERROR_TERMINATE(rox_array2d_double_new(&Hpp_inverse, 3, 3));
   CHECK_ERROR_TERMINATE(rox_array2d_double_new(&Hpctp, 6, 1));
   CHECK_ERROR_TERMINATE(rox_array2d_double_new(&xcam, sizecols_cam, 1));
   CHECK_ERROR_TERMINATE(rox_array2d_double_new(&tmp3, 3, 1));
   CHECK_ERROR_TERMINATE(rox_array2d_double_new(&xcam_buffer, 6, 1));


   error = rox_array2d_double_get_data_pointer_to_pointer(&da, A);
   error = rox_array2d_double_get_data_pointer(&db, B);
   error = rox_array2d_double_get_data_pointer(&dhpctp, Hpctp);

   error = rox_array2d_double_fillunit(A);
   error = rox_array2d_double_fillunit(B);

   // Build frame-frame hessian

   poscam = 0;
   for (idframe = 0; idframe < obj->frames->used; idframe++)
   {
      Rox_Bundle_Frame frame;
      Rox_Double ** dh, *de;

      frame = obj->frames->data[idframe];
      if (frame->is_invalid) continue;
      if (frame->is_fixed) continue;

      // Backup frame pose, to restore it after update if needed.

      error = rox_array2d_double_copy(frame->pose_previous, frame->pose);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_get_data_pointer_to_pointer ( &dh, frame->hessian);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_get_data_pointer ( &de, frame->projected_error);
      ROX_ERROR_CHECK_TERMINATE ( error );

      for (i = 0; i < 6; i++)
      {
         int poscami = poscam + i;

         for (j = 0; j < 6; j++)
         {
            da[poscami][poscam + j] = dh[i][j];
         }

         db[poscami] = de[i];

         // Update diagonal
         da[poscami][poscam + i] *= (1.0 + obj->lambda);
      }

      frame->pos = poscam;
      poscam += 6;
   }


   // Backup points to restore them if needed after update

   for (idpoint = 0; idpoint < obj->points->used; idpoint++)
   {
      if (obj->points->data[idpoint]->is_invalid) continue;
      obj->points->data[idpoint]->coords_previous = obj->points->data[idpoint]->coords;
   }

   // Build tracks

   for (idpoint = 0; idpoint < obj->points->used; idpoint++)
   {
      Rox_Bundle_Point point;
      Rox_Uint poscam;

      point = obj->points->data[idpoint];
      if (point->is_invalid) continue;

      Hpp = point->hessian;
      error = rox_array2d_double_get_data_pointer_to_pointer(&dhpp, Hpp);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Update diagonal
      for (i = 0; i < 3; i++)
      {
         dhpp[i][i] = (1.0 + obj->lambda) * dhpp[i][i];
      }

      // Inverse point hessian
      error = rox_array2d_double_svdinverse(Hpp_inverse, Hpp);
      if (error)
      {
         rox_array2d_double_fillval(Hpp_inverse, 0);
         rox_array2d_double_fillval(point->tp, 0);
         point->is_invalid = 1;
         continue;
      }

      // Hpp^+ * err
      error = rox_array2d_double_mulmatmat(point->tp, Hpp_inverse, point->projected_error);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // For each measure of this points
      for (idmes = 0; idmes < point->measures->used; idmes++)
      {
         Rox_Bundle_Measure mes;
         Rox_Bundle_Frame frame;
         Rox_Double ** dtpc;

         mes = point->measures->data[idmes];
         if (mes->is_invalid) continue;

         frame = mes->frame;
         if (frame->is_invalid) continue;
         if (frame->is_fixed) continue;

         poscam = frame->pos;

         error = rox_array2d_double_mulmattransmat(Hpctp, mes->JpointTJframe, point->tp);
         if (error) continue;

         error = rox_array2d_double_mulmattransmat(mes->Tpc, mes->JpointTJframe, Hpp_inverse);
         if (error) continue;

         for (i = 0; i < 6; i++)
         {
            db[poscam + i] -= dhpctp[i];
         }

         error = rox_array2d_double_get_data_pointer_to_pointer( &dtpc, mes->Tpc);
         if (error) continue;


         for (idmes2 = 0; idmes2 < point->measures->used; idmes2++)
         {
            Rox_Bundle_Measure mes2;
            Rox_Bundle_Frame frame2;
            Rox_Double ** djpf;
            Rox_Double cell;

            mes2 = point->measures->data[idmes2];
            if (mes2->is_invalid) continue;

            frame2 = mes2->frame;
            if (frame2->is_invalid) continue;
            if (frame2->is_fixed) continue;

            poscam2 = frame2->pos;

            if (poscam2 < poscam) continue;

            error = rox_array2d_double_get_data_pointer_to_pointer( &djpf, mes2->JpointTJframe);

            for (i = 0; i < 6; i++)
            {
               rowa = &da[poscam + i][poscam2];

               for (j = 0; j < 6; j++)
               {
                  cell = 0;

                  for(k = 0; k < 3; k++)
                  {
                     cell += dtpc[i][k] * djpf[k][j];
                  }

                  rowa[j] -= cell;
               }
            }
         }
      }
   }

   for (i = 0; i < sizecols_cam; i++)
   {
      for (j = i; j < sizecols_cam; j++)
      {
         da[j][i] = da[i][j];
      }
   }

   // Solve for camera parameters
   if (obj->count_valid_frames > 0)
   {
      error = rox_array2d_double_cholesky_decomposition(A_lot, A);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_lotinverse(A_inverse, A_lot);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_mulmatmat(xcam, A_inverse, B);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   // Use this solution to update the frames

   for (idframe = 0; idframe < obj->frames->used; idframe++)
   {
      Rox_Double * dx;
      Rox_Bundle_Frame frame;
      Rox_Array2D_Double subxcam;
      frame = obj->frames->data[idframe];

      // Do not update fixed frames
      if (frame->is_invalid) continue;
      if (frame->is_fixed) continue;

      poscam = frame->pos;
      subxcam = NULL;

      // Get subvector

      error = rox_array2d_double_new_subarray2d(&subxcam, xcam, poscam, 0, 6, 1);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Save update parameters for LM

      error = rox_array2d_double_get_data_pointer(&dx, subxcam);
      ROX_ERROR_CHECK_TERMINATE ( error );

      frame->update[0] = dx[0];
      frame->update[1] = dx[1];
      frame->update[2] = dx[2];
      frame->update[3] = dx[3];
      frame->update[4] = dx[4];
      frame->update[5] = dx[5];


      // Update camera pose
      rox_array2d_double_scale(xcam_buffer, subxcam, -1.0);
      rox_matse3_update_right(frame->pose, xcam_buffer);

      rox_array2d_double_del(&subxcam);
   }


   // Propagate back solution to points

   for (idpoint = 0; idpoint < obj->points->used; idpoint++)
   {
      Rox_Bundle_Point point;
      Rox_Array2D_Double dp;
      Rox_Bundle_Measure mes;
      Rox_Bundle_Frame frame;
      Rox_Array2D_Double subxcam;
      Rox_Double * ddp;

      point = obj->points->data[idpoint];

      if (point->is_invalid) continue;

      dp = point->tp;

      for (idmes = 0; idmes < point->measures->used; idmes++)
      {
         mes = point->measures->data[idmes];
         if (mes->is_invalid) continue;

         frame = mes->frame;
         if (frame->is_invalid) continue;
         if (frame->is_fixed) continue;

         poscam = frame->pos;

         subxcam = NULL;

         error = rox_array2d_double_new_subarray2d(&subxcam, xcam, poscam, 0, 6, 1);
         ROX_ERROR_CHECK_TERMINATE(error)

         error = rox_array2d_double_mulmattransmat(tmp3, mes->Tpc, subxcam);
         ROX_ERROR_CHECK_TERMINATE(error)

         error = rox_array2d_double_substract(dp, dp, tmp3);
         ROX_ERROR_CHECK_TERMINATE(error)

         rox_array2d_double_del(&subxcam);
      }

      error = rox_array2d_double_get_data_pointer( &ddp, dp);

      point->update[0] = ddp[0];
      point->update[1] = ddp[1];
      point->update[2] = ddp[2];

      point->coords.X -= point->update[0];
      point->coords.Y -= point->update[1];
      point->coords.Z -= point->update[2];
   }


function_terminate:
   rox_array2d_double_del(&A);
   rox_array2d_double_del(&A_lot);
   rox_array2d_double_del(&A_inverse);
   rox_array2d_double_del(&B);
   rox_array2d_double_del(&Hpp_chol);
   rox_array2d_double_del(&Hpp_inverse);
   rox_array2d_double_del(&Hpctp);
   rox_array2d_double_del(&xcam);
   rox_array2d_double_del(&tmp3);
   rox_array2d_double_del(&xcam_buffer);
   return error;
}

Rox_ErrorCode rox_bundle_check_thresholds(Rox_Bundle obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double maxval = 0.0;

   for (Rox_Uint idpt = 0; idpt < obj->points->used; idpt++)
   {
      if (obj->points->data[idpt]->is_invalid) continue;

      Rox_Double * dg = NULL;
      error = rox_array2d_double_get_data_pointer( &dg, obj->points->data[idpt]->projected_error);

      for ( Rox_Sint pos = 0; pos < 3; pos++)
      {
         if (dg[pos] > maxval) maxval = dg[pos];
      }
   }

   for (Rox_Uint idframe = 0; idframe < obj->frames->used; idframe++)
   {
      if (obj->frames->data[idframe]->is_invalid) continue;
      if (obj->frames->data[idframe]->is_fixed) continue;
      Rox_Double * dg = NULL;
      error = rox_array2d_double_get_data_pointer ( &dg, obj->frames->data[idframe]->projected_error);

      for ( Rox_Sint pos = 0; pos < 6; pos++)
      {
         if (dg[pos] > maxval) maxval = dg[pos];
      }
   }

   if (maxval < obj->eps2) {error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE(error)}

function_terminate:
   return error;
}

Rox_ErrorCode rox_bundle_check_convergence(Rox_Bundle obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double delta_error, norm;
   Rox_Double *dg;
   Rox_Uint idpt, idframe;

   obj->norm_error_previous = obj->norm_error;

   error = rox_bundle_compute_prediction(obj);
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_bundle_check_validity(obj);
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_bundle_compute_weights(obj);
   ROX_ERROR_CHECK_TERMINATE ( error );

   delta_error = obj->norm_error_previous - obj->norm_error;

   norm = 0.0;
   for (idpt = 0; idpt < obj->points->used; idpt++)
   {
      if (obj->points->data[idpt]->is_invalid) continue;

      error = rox_array2d_double_get_data_pointer ( &dg, obj->points->data[idpt]->projected_error);

      norm += obj->points->data[idpt]->update[0] * (obj->lambda * obj->points->data[idpt]->update[0] + dg[0]);
      norm += obj->points->data[idpt]->update[1] * (obj->lambda * obj->points->data[idpt]->update[1] + dg[1]);
      norm += obj->points->data[idpt]->update[2] * (obj->lambda * obj->points->data[idpt]->update[2] + dg[2]);
   }

   for (idframe = 0; idframe < obj->frames->used; idframe++)
   {
      if (obj->frames->data[idframe]->is_invalid) continue;
      if (obj->frames->data[idframe]->is_fixed) continue;

      error = rox_array2d_double_get_data_pointer( &dg, obj->frames->data[idframe]->projected_error);

      norm += obj->frames->data[idframe]->update[0] * (obj->lambda * obj->frames->data[idframe]->update[0] + dg[0]);
      norm += obj->frames->data[idframe]->update[1] * (obj->lambda * obj->frames->data[idframe]->update[1] + dg[1]);
      norm += obj->frames->data[idframe]->update[2] * (obj->lambda * obj->frames->data[idframe]->update[2] + dg[2]);
      norm += obj->frames->data[idframe]->update[3] * (obj->lambda * obj->frames->data[idframe]->update[3] + dg[3]);
      norm += obj->frames->data[idframe]->update[4] * (obj->lambda * obj->frames->data[idframe]->update[4] + dg[4]);
      norm += obj->frames->data[idframe]->update[5] * (obj->lambda * obj->frames->data[idframe]->update[5] + dg[5]);
   }

   obj->rho = delta_error / norm;
   if (obj->rho <= 0)
   {
      for (idpt = 0; idpt < obj->points->used; idpt++)
      {
         if (obj->points->data[idpt]->is_invalid) continue;

         obj->points->data[idpt]->coords = obj->points->data[idpt]->coords_previous;
      }

      for (idframe = 0; idframe < obj->frames->used; idframe++)
      {
         if (obj->frames->data[idframe]->is_invalid) continue;
         if (obj->frames->data[idframe]->is_fixed) continue;
         rox_array2d_double_copy(obj->frames->data[idframe]->pose, obj->frames->data[idframe]->pose_previous);
      }
   }

   error = rox_bundle_compute_prediction(obj);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_bundle_check_validity(obj);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_bundle_compute_weights(obj);
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (obj->rho > 0)
   {
      obj->lambda = ROX_MAX(1e-10, obj->lambda * 0.1);
      //Rox_Double val;
      //val = 1.0 - pow(2.0 * obj->rho - 1.0, 3);
      //val = ROX_MAX(1.0/3.0, val);
      //obj->lambda = obj->lambda * val;
      //obj->lambda_multiplier = 2.0;
   }
   else
   {
      obj->lambda *= 10.0;
      //obj->lambda = obj->lambda * obj->lambda_multiplier;
      //obj->lambda_multiplier *= 2.0;
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_bundle_optimize(Rox_Bundle obj, Rox_Uint max_iterations)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint iter = 0;
   Rox_Double delta_error;
   Rox_Uint count_nomove;

   if (!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   obj->lambda_multiplier = 2.0;
   obj->norm_error_previous = 0.0;
   obj->norm_error = 0.0;

   // Compute initial state
   error = rox_bundle_compute_prediction(obj);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_bundle_check_validity(obj);
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_bundle_compute_weights(obj);
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_bundle_compute_hessians(obj);
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_bundle_compute_initial_lambda(obj);
   ROX_ERROR_CHECK_TERMINATE ( error );

   count_nomove = 0;

   obj->max_iterations = max_iterations;

   while (iter < obj->max_iterations)
   {
      error = rox_bundle_solve_system(obj);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_bundle_check_convergence(obj);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_bundle_compute_hessians(obj);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Update thresholds
      error = rox_bundle_check_thresholds(obj);
      if (error)
      {
         // Err means we can break early
         error = ROX_ERROR_NONE;
         break;
      }

      delta_error = fabs(obj->norm_error - obj->norm_error_previous);
      if (delta_error < 1e-8)
      {
         count_nomove++;
      }
      else
      {
         count_nomove = 0;
      }

      if (count_nomove > 3)
      {
         break;
      }

      iter++;
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_bundle_get_point_result(Rox_Point3D_Double  coords, Rox_Bundle obj, Rox_Uint id)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!obj || !coords) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   if (id >= obj->points->used) {error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE(error)}

   if (obj->points->data[id]->is_invalid) {error = ROX_ERROR_INVALID; ROX_ERROR_CHECK_TERMINATE(error)}

   *coords = obj->points->data[id]->coords;

function_terminate:
    return error;
}

Rox_ErrorCode rox_bundle_get_pose_result(Rox_Array2D_Double pose, Rox_Bundle obj, Rox_Uint id)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!obj || !pose) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   if (id >= obj->frames->used) {error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE(error)}

   if (obj->frames->data[id]->is_invalid) {error = ROX_ERROR_INVALID; ROX_ERROR_CHECK_TERMINATE(error)}

   error = rox_array2d_double_copy(pose, obj->frames->data[id]->pose);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
    return error;
}

Rox_ErrorCode rox_bundle_get_measure_validity(Rox_Uint * validity, Rox_Bundle obj, Rox_Uint id)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!obj || !validity) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   if (id >= obj->measures->used) {error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE(error)}

   if (obj->measures->data[id]->is_invalid)
   {
      *validity = 0;
   }
   else
   {
      *validity = 1;
   }

function_terminate:
    return error;
}
